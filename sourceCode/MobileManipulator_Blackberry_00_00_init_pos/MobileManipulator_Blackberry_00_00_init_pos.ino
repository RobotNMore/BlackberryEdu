//////////////  픽시 카메라 모듈에 연결된 그리퍼를 서서히 오므렸다 벌리기
#include <Pixy2SPI_SS.h>

Pixy2SPI_SS pixy;

#define GRIP_ANGLE_CLOSE          650  // 그리퍼 반 정도 닫히는 값(1000이 최대)
#define GRIP_ANGLE_OPEN           100  // 그리퍼 열리는 값


//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1

const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////  ARM MOTORS
////////////////////////////////////////////////////////////////////////////

//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL            0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR            0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL            0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR            0x04 // 우측 후방 모터 아이디

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드 ... mobile/arm 공통
#define REVERSE_MODE                0x01

// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = {MOBILE_DXL_ID_FL,
                                                   MOBILE_DXL_ID_FR,
                                                   MOBILE_DXL_ID_BL,
                                                   MOBILE_DXL_ID_BR};

// sync write 용 상수, 구조체 정의 및 인스턴스화
const int32_t GOAL_VELOCITY_VALUE = 88; // 모터에 적용할 속도 값

const uint16_t GOAL_VELOCITY_ADDR = 104; // sync write start 주소
const uint16_t GOAL_VELOCITY_SIZE = 4; // sync write data 길이

typedef struct mobile_sw_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) mobile_sw_data_t;

mobile_sw_data_t mobile_sw_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t mobile_sw_infos; // syncwrite 정보 인스턴스 생성

// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t mobile_info_xels_sw[MOBILE_DXL_ID_CNT];

/*
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 */
void SetMobileGoalVelocityForSyncWrite(int32_t fl_goal_velocity,
                                       int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity,
                                       int32_t br_goal_velocity) {
  mobile_sw_data[0].goal_velocity = fl_goal_velocity;
  mobile_sw_data[1].goal_velocity = fr_goal_velocity;
  mobile_sw_data[2].goal_velocity = bl_goal_velocity;
  mobile_sw_data[3].goal_velocity = br_goal_velocity;
  mobile_sw_infos.is_info_changed = true;
}

////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////  ARM MOTORS
////////////////////////////////////////////////////////////////////////////

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
#define ARM_DXL_2_POSITION_MAX  3474
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

// sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t ARM_SW_START_ADDR = 112; // sync write start 주소
const uint16_t ARM_SW_DATA_SIZE = 8; // sync write data 길이

typedef struct arm_sw_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) arm_sw_data_t;

arm_sw_data_t arm_sw_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성

DYNAMIXEL::InfoSyncWriteInst_t arm_sw_infos; // syncwrite 정보 인스턴스 생성

// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t arm_info_xels_sw[ARM_DXL_ID_CNT];


//////////////  매니퓰레이터 계산용 변수/상수
#define pi      3.141592
#define DTR(x)  (x)*(pi/180) // degree to radian
#define RTD(x)  (x)*(180/pi) // radian to degree

const float d = 79.75; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 109.21; // 2번과 3번 모터의 회전축간 거리
const float L2 = 86.4; // 3번과 4번 모터의 회전축간 거리
const float L3 = 97.2; // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리

#define MANIPULATOR_X_UPPER_LIMIT       90    // 매니퓰레이터 X 상한 값
#define MANIPULATOR_X_DEFAULT           0     // 매니퓰레이터 X 기본 값
#define MANIPULATOR_X_LOWER_LIMIT       -90   // 매니퓰레이터 X 하한 값

#define MANIPULATOR_Y_DEFAULT           130   // 매니퓰레이터 Y 기본 값

#define MANIPULATOR_Z_UPPER_LIMIT       250   // 매니퓰레이터 Z 상한 값
#define MANIPULATOR_Z_DEFAULT           190   // 매니퓰레이터 Z 기본 값
#define MANIPULATOR_Z_LOWER_LIMIT       60    // 매니퓰레이터 Z 하한 값

#define MANIPULATOR_ANGLE_DEFAULT       5     // 기본 그리퍼 피치각

#define PIXY2_X_SETPOINT                158   // 카메라 x위치 목표값
#define PIXY2_Y_SETPOINT                104   // 카메라 y위치 목표값

int16_t X = MANIPULATOR_X_DEFAULT;
int16_t Y = MANIPULATOR_Y_DEFAULT;
int16_t Z = MANIPULATOR_Z_DEFAULT;
int16_t Angle = MANIPULATOR_ANGLE_DEFAULT;

///////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////

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
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고
    // 0~4095 범위로 매핑
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
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
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
    arm_sw_data[0].goal_position = motor1GoalPosition;
    arm_sw_data[0].profile_velocity = operatingTime;
  }
  if (motor2GoalPosition != -1) {
    arm_sw_data[1].goal_position = motor2GoalPosition;
    arm_sw_data[1].profile_velocity = operatingTime;
  }
  if (motor3GoalPosition != -1) {
    arm_sw_data[2].goal_position = motor3GoalPosition;
    arm_sw_data[2].profile_velocity = operatingTime;
  }
  if (motor4GoalPosition != -1) {
    arm_sw_data[3].goal_position = motor4GoalPosition;
    arm_sw_data[3].profile_velocity = operatingTime;
  }
  
  arm_sw_infos.is_info_changed = true;
}


/*
 * 그리퍼의 위치와 그리퍼의 피치 각도로 4-DOF 매니퓰레이터를 제어하는 함수
 * params :
 *    x : 그리퍼 좌우 위치(mm)
 *    y : 그리퍼 전후 위치(mm)
 *    z : 그리퍼 높이(mm)
 *    angle : 그리퍼 피치 각도(degree)
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void SetManipulatorInverseMoveForSyncWrite( int16_t x, int16_t y, int16_t z,
                                            int16_t angle, int32_t operatingTime ) {
  int16_t zFromMotor2 = z - d; // 계산의 편의를 위해 2번 모터 관절과 지면 사이의 거리를 무시한 z축(위/아래) 거리
  
  // 매니퓰레이터를 측면에서 봤을 때 전진 거리 yd
  float yd = sqrt((int32_t)x*x + (int32_t)y*y)*(y/abs(y));
  
  // 매니퓰레이터를 측면에서 봤을 때 2번째 링크가 끝나는 점의 위치 (y2, z2)
  float y2 = yd - L3*cos(DTR(angle));
  float z2 = zFromMotor2 - L3*sin(DTR(angle));
  
  // 세타1을 구하기 위해 사용되는 변수
  float k1 = L1 + L2*(y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2);
  float k2 = L2*-1*sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2));
  float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0; // 모터별 세타 변수

  theta1 = RTD(atan2(y, x)); // atan2는 -pi~pi 범위로 결과를 반환 함
  theta2 = RTD(atan2(z2, y2)) - RTD(atan2(k2, k1));
  theta3 = RTD(atan2(-1*sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2)),
                     (y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2)));
  theta4 = angle - theta2 - theta3;

  float a1 = 90 - theta1; // 세타 값을 모터 제어를 위한 각도로 변환
  if (a1 > 180) // 범위를 -180~180으로 변환
    a1 -= 360;
  float a2 = -90 + theta2;
  if (a2 < -180)
    a2 += 360;
  float a3 = theta3;
  float a4 = theta4;

  // 계산된 모터 각도로 매니퓰레이터 동작 준비
  SetManipulatorForwardMoveForSyncWrite( a1, a2, a3, a4, operatingTime);

  Serial.println("Result");
  Serial.print("    theta1 = ");
  Serial.println(theta1);
  Serial.print("    theta2 = ");
  Serial.println(theta2);
  Serial.print("    theta3 = ");
  Serial.println(theta3);
  Serial.print("    theta4 = ");
  Serial.println(theta4);
  Serial.println();

  Serial.println("Result motor angle");
  Serial.print("    m1angle = ");
  Serial.println(a1);
  Serial.print("    m2angle = ");
  Serial.println(a2);
  Serial.print("    m3angle = ");
  Serial.println(a3);
  Serial.print("    m4angle = ");
  Serial.println(a4);
}


///////////////////////////////////////////////////////////

//////////////  메인 프로그램

void setup()
{
  pinMode(4, OUTPUT);

  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindManipulatorServos()) {}


  /////////////////////////////////////////////////////////////////////

  // 각 MOBILE 모터에 설정
  for (int i = 0 ; i < sizeof(MOBILE_DXL_IDS)/sizeof(uint8_t) ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], 0);
  
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], 1);

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], 1);
  }

  // sync write 준비
  mobile_sw_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  mobile_sw_infos.packet.is_completed = false; // false로 초기화
  mobile_sw_infos.addr = GOAL_VELOCITY_ADDR; // 컨트롤 테이블에 sync write를 시작하는 주소
  mobile_sw_infos.addr_length = GOAL_VELOCITY_SIZE; // sync write하는 데이터 길이
  mobile_sw_infos.p_xels = mobile_info_xels_sw; // sync write 할 모터 정보
  mobile_sw_infos.xel_count = 0; // sync write 할 모터 개수

  mobile_sw_data[0].goal_velocity = 0; // 모터에 write 할 데이터 초기화
  mobile_sw_data[1].goal_velocity = 0;
  mobile_sw_data[2].goal_velocity = 0;
  mobile_sw_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    mobile_info_xels_sw[i].id = MOBILE_DXL_IDS[i];
    mobile_info_xels_sw[i].p_data = (uint8_t*)&mobile_sw_data[i].goal_velocity;
    mobile_sw_infos.xel_count++;
  }
  mobile_sw_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정 

  SetMobileGoalVelocityForSyncWrite(0, 0, 0, 0);
  while(dxl.syncWrite(&mobile_sw_infos) == false) { delay(10); }


  ////////////////////////////////////////////////////////////////////////

  // 각 ARM 모터에 설정
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
  arm_sw_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  arm_sw_infos.packet.is_completed = false; // false로 초기화
  arm_sw_infos.addr = ARM_SW_START_ADDR; // 컨트롤 테이블에 sync write를 시작하는 주소
  arm_sw_infos.addr_length = ARM_SW_DATA_SIZE; // sync write하는 데이터 길이
  arm_sw_infos.p_xels = arm_info_xels_sw; // sync write 할 모터 정보
  arm_sw_infos.xel_count = 0; // sync write 할 모터 개수

  arm_sw_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  arm_sw_data[0].goal_position = 2048;
  arm_sw_data[1].profile_velocity = 1000;
  arm_sw_data[1].goal_position = 2048;
  arm_sw_data[2].profile_velocity = 1000;
  arm_sw_data[2].goal_position = 2048;
  arm_sw_data[3].profile_velocity = 1000;
  arm_sw_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    arm_info_xels_sw[i].id = ARM_DXL_IDS[i];
    arm_info_xels_sw[i].p_data = (uint8_t*)&arm_sw_data[i];
    arm_sw_infos.xel_count++;
  }
  arm_sw_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정

  SetManipulatorInverseMoveForSyncWrite(X, Y, Z, Angle, 1000);
  while(!dxl.syncWrite(&arm_sw_infos)) {}

  ///////////////////////////////////////////////////////////////////////

  pixy.init(); // 카메라 모듈 초기화
  
  pixy.setServos(0, GRIP_ANGLE_OPEN); // 그리퍼 열기
  delay(1000); // 움직임 시간 대기
}

void loop()
{
  digitalWrite(4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);
  digitalWrite(4, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);
}

