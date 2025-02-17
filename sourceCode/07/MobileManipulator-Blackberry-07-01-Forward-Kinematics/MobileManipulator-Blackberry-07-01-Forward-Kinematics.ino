//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1

const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  모터 구동용 변수/상수
#define ARM_DXL_ID_1            0x05 // 매니퓰레이터 1번 모터 아이디 (가장 아래)
#define ARM_DXL_ID_2            0x06 // 매니퓰레이터 2번 모터 아이디
#define ARM_DXL_ID_3            0x07 // 매니퓰레이터 3번 모터 아이디
#define ARM_DXL_ID_4            0x08 // 매니퓰레이터 4번 모터 아이디 (가장 위)

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_BASED_PROFILE      0x00 // profile configuration
#define TIME_BASED_PROFILE          0x04

#define ARM_DXL_1_POSITION_MIN  0
#define ARM_DXL_1_POSITION_MAX  4095
#define ARM_DXL_2_POSITION_MIN  1024
#define ARM_DXL_2_POSITION_MAX  3071
#define ARM_DXL_3_POSITION_MIN  211
#define ARM_DXL_3_POSITION_MAX  2640
#define ARM_DXL_4_POSITION_MIN  722
#define ARM_DXL_4_POSITION_MAX  3161

#define ARM_DXL_1_OFFSET        0
#define ARM_DXL_2_OFFSET        238
#define ARM_DXL_3_OFFSET        799
#define ARM_DXL_4_OFFSET        0

// FinManipulatorServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t ARM_DXL_ID_CNT = 4;
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = {ARM_DXL_ID_1, ARM_DXL_ID_2, ARM_DXL_ID_3, ARM_DXL_ID_4};

// sync write용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_START_ADDR = 112; // sync write start 주소
const uint16_t SW_DATA_SIZE = 8;    // sync write data 길이

typedef struct sw_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성

DYNAMIXEL::InfoSyncWriteInst_t sw_infos; // syncwrite 정보 인스턴스 생성

// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[ARM_DXL_ID_CNT];


//////////////  매니퓰레이터 계산용 변수/상수

#define pi      3.141592
#define DTR(x)  (x)*(pi/180) // degree to radian
// rad = 180deg / pi =~ 57.3deg, deg = pi / 180rad =~ 0.0175rad

float a1 = 0;  // (+)= 우측, (-)= 좌측
float a2 = 0;  // (+)= 뒤로, (-)= 앞으로
float a3 = 0;  // (+)= 뒤로, (-)= 앞으로
float a4 = 0;  // (+)= 뒤로, (-)= 앞으로

//float a1 = 45;
//float a2 = 45;
//float a3 = -90;
//float a4 = -45;

const float  d =  79.75; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 109.21; // 2번과 3번 모터의 회전축간 거리
const float L2 =  86.4;  // 3번과 4번 모터의 회전축간 거리
const float L3 =  97.2;  // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리

float X = 0;  // 모터 각도에 따른 그리퍼의 위치 결과 값
float Y = 0;
float Z = 0;
float Yd = 0; // 매니퓰레이터 단면에서 수평방향 길이


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindManipulatorServos()) {}

  // 각 모터에 설정
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 0);
  
    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Time-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i], REVERSE_MODE + TIME_BASED_PROFILE);
    
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], 3);

    switch (ARM_DXL_IDS[i]) { // 모터에 따라 position limit, homing offset 설정
      case ARM_DXL_ID_1:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_1_OFFSET);
        break;
      case ARM_DXL_ID_2:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_2_OFFSET);
        break;
      case ARM_DXL_ID_3:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_3_OFFSET);
        break;
      case ARM_DXL_ID_4:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,      ARM_DXL_IDS[i], ARM_DXL_4_OFFSET);
        break;
    }

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 1);
  }

  // sync write 준비
  sw_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_infos.packet.is_completed = false; // false로 초기화
  sw_infos.addr = SW_START_ADDR; // 컨트롤 테이블에 sync write를 시작하는 주소
  sw_infos.addr_length = SW_DATA_SIZE; // sync write하는 데이터 길이
  sw_infos.p_xels = info_xels_sw; // sync write 할 모터 정보
  sw_infos.xel_count = 0; // sync write 할 모터 개수

  sw_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  sw_data[0].goal_position = 2048;
  sw_data[1].profile_velocity = 1000;
  sw_data[1].goal_position = 2048;
  sw_data[2].profile_velocity = 1000;
  sw_data[2].goal_position = 2048;
  sw_data[3].profile_velocity = 1000;
  sw_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_xels_sw[i].id = ARM_DXL_IDS[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i];
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정

  SetManipulatorForwardMoveForSyncWrite( a1, a2, a3, a4, 2000 );
  while(!dxl.syncWrite(&sw_infos)) {}

  // 모터각도를 그리퍼 위치 계산을 위한 세타값(좌표각도)으로 변환
  float theta1 = 90 - a1;
  float theta2 = 90 + a2;
  float theta3 = a3;
  float theta4 = a4;

  // 좌표 계산
  Yd = L1*cos(DTR(theta2))
       + L2*cos(DTR(theta2) + DTR(theta3))
       + L3*cos(DTR(theta2) + DTR(theta3) + DTR(theta4));
       
  Z = L1*sin(DTR(theta2))
      + L2*sin(DTR(theta2) + DTR(theta3))
      + L3*sin(DTR(theta2) + DTR(theta3) + DTR(theta4))
      + d;
      
  X = Yd*cos(DTR(theta1));
  
  Y = Yd*sin(DTR(theta1));

  Serial.println("---------------------------------------------\n");

  Serial.print("manipulator motor1 angle : ");
  Serial.println(a1);
  Serial.print("manipulator motor2 angle : ");
  Serial.println(a2);
  Serial.print("manipulator motor3 angle : ");
  Serial.println(a3);
  Serial.print("manipulator motor4 angle : ");
  Serial.println(a4);
  Serial.println();

  Serial.print("theta1 : "); // 매니퓰레이터 모터 1번의 각도
  Serial.println(theta1);
  Serial.print("theta2 : "); // 매니퓰레이터 모터 2번의 지면과의 각도
  Serial.println(theta2);
  Serial.print("theta3 : "); // 매니퓰레이터 모터 3번의 각도
  Serial.println(theta3);
  Serial.print("theta4 : "); // 매니퓰레이터 모터 4번의 각도
  Serial.println(theta4);
  Serial.println();

  Serial.print("gripper position yd : ");
  Serial.println(Yd);
  Serial.println();
  
  Serial.print("gripper position x : ");
  Serial.println(X);
  Serial.print("gripper position y : ");
  Serial.println(Y);
  Serial.print("gripper position z : ");
  Serial.println(Z);
  Serial.println();
  Serial.println("---------------------------------------------\n");

  Serial.println("\n");
}

void loop()
{
}

/*
 * ARM_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수. 모두 통신 가능하면 true, 아니면 false를 반환
 */
bool FindManipulatorServos() {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= sizeof(ARM_DXL_IDS)/sizeof(uint8_t)) {
      uint8_t arm_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (ARM_DXL_IDS[arm_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          arm_dxl_ids_idx ++;

          if (arm_dxl_ids_idx
              == sizeof(ARM_DXL_IDS)/sizeof(uint8_t)) {
            // 찾으려는 모터를 모두 찾은 경우
            break;
          }
        } else {
          if (ids_pinged_idx == count_pinged) {
             // 통신가능한 모터가 더이상 없는 경우
             is_each_motor_found = false;
             break;
          }
        }
      }
      
      if (!is_each_motor_found) {
        Serial.print("Motor IDs does not match : ");
        Serial.println(dxl.getLastLibErrCode());
      }
    } else {
      Serial.print("Motor count does not match : ");
      Serial.println(dxl.getLastLibErrCode());
      is_each_motor_found = false;
    }
  } else{
    Serial.print("Broadcast returned no items : ");
    Serial.println(dxl.getLastLibErrCode());
    is_each_motor_found = false;
  }
  return is_each_motor_found;
}

/* 
 * 각도를 입력받아 모터의 위치 값을 반환하는 함수
 * argument : 
 *    angle : 모터의 목표 각도
 * return :
 *    대응하는 모터의 목표 위치 값. -1은 변환 실패
 */
int32_t GetArmServoGoalPositionWithAngle(float angle) {
  if (!isnan(angle)) {
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고, 0~4095 범위로 매핑
    return (int32_t)map(constrain((int32_t)angle, -180, 180), -180, 180, 0, 4095);
  } else {
    return (int32_t)-1;
  }
}

/* 
 * 매니퓰레이터를 네 모터의 각도 값과 시간으로 제어를 준비하는 함수
 * argument : 
 *    a1 : 5번 모터 각도
 *    a2 : 6번 모터 각도
 *    a3 : 7번 모터 각도
 *    a4 : 8번 모터 각도
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 걸릴 시간
 */
void SetManipulatorForwardMoveForSyncWrite( float a1,
                                            float a2,
                                            float a3,
                                            float a4,
                                            int32_t operatingTime) {
  int32_t motor1GoalPosition = GetArmServoGoalPositionWithAngle( a1 );
  int32_t motor2GoalPosition = GetArmServoGoalPositionWithAngle( a2 );
  int32_t motor3GoalPosition = GetArmServoGoalPositionWithAngle( a3 );
  int32_t motor4GoalPosition = GetArmServoGoalPositionWithAngle( a4 );
  
  if (motor1GoalPosition != -1) {
    sw_data[0].goal_position = motor1GoalPosition;
    sw_data[0].profile_velocity = operatingTime;
  }
  if (motor2GoalPosition != -1) {
    sw_data[1].goal_position = motor2GoalPosition;
    sw_data[1].profile_velocity = operatingTime;
  }
  if (motor3GoalPosition != -1) {
    sw_data[2].goal_position = motor3GoalPosition;
    sw_data[2].profile_velocity = operatingTime;
  }
  if (motor4GoalPosition != -1) {
    sw_data[3].goal_position = motor4GoalPosition;
    sw_data[3].profile_velocity = operatingTime;
  }
  
  sw_infos.is_info_changed = true;
}
