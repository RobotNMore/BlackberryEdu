//////////////  [실습] 원근법을 깨우쳐 밀당하는 블랙 베리
// 기준 물체의 크기 변화를 이용해 모바일 베이스가 전진과 후진을 실행 
// 기준 거리(30cm)에서의 물체의 면적을 측정하여,
// 면적 변화량(기준면적(PIXY2_DEFAULT_AREA=800)-측정면적)에 비례하여 주행 속도를 변화시킴

//////////////  Pixy2 카메라
#include <Pixy2SPI_SS.h>        // SPI 통신하는 Pixy2 라이브러리를 포함시킴
Pixy2SPI_SS pixy;               // SPI 통신하는 Pixy2 객체 생성

//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL        0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR        0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL        0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR        0x04 // 우측 후방 모터 아이디
#define ARM_DXL_ID_1            0x05 // 매니퓰레이터 1번 모터 아이디 (가장 아래)
#define ARM_DXL_ID_2            0x06 // 매니퓰레이터 2번 모터 아이디
#define ARM_DXL_ID_3            0x07 // 매니퓰레이터 3번 모터 아이디
#define ARM_DXL_ID_4            0x08 // 매니퓰레이터 4번 모터 아이디 (가장 위)

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_BASED_PROFILE      0x00 // profile configuration
#define TIME_BASED_PROFILE          0x04


// FinManipulatorServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t ARM_DXL_ID_CNT = 4;
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = {ARM_DXL_ID_1,
                                             ARM_DXL_ID_2,
                                             ARM_DXL_ID_3,
                                             ARM_DXL_ID_4};
const uint32_t ARM_DXL_POSITION_MIN[ARM_DXL_ID_CNT] = {0, 1024, 211, 722};
const uint32_t ARM_DXL_POSITION_MAX[ARM_DXL_ID_CNT] = {4095, 3474, 2640, 3161};
const uint32_t ARM_DXL_OFFSET[ARM_DXL_ID_CNT] = {0, 238, 799, 0};


// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = {MOBILE_DXL_ID_FL,
                                                   MOBILE_DXL_ID_FR,
                                                   MOBILE_DXL_ID_BL,
                                                   MOBILE_DXL_ID_BR};

// arm sync write 용 상수, 구조체 정의 및 인스턴스화
const uint16_t SW_JOINT_START_ADDR = 112; // sync write start 주소
const uint16_t SW_JOINT_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_joint_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_joint_data_t;

sw_joint_data_t sw_joint_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_arm_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_arm_xels_sw[ARM_DXL_ID_CNT];

// mobile sync write 용 상수, 구조체 정의 및 인스턴스화
const int32_t GOAL_VELOCITY_VALUE = 88; // 모터에 적용할 속도 값
const uint16_t SW_WHEEL_START_ADDR = 104; // sync write start 주소
const uint16_t SW_WHEEL_DATA_SIZE = 4; // sync write data 길이
typedef struct sw_wheel_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) sw_wheel_data_t;

sw_wheel_data_t sw_wheel_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_mobile_xels_sw[MOBILE_DXL_ID_CNT];


//////////////  매니퓰레이터 계산용 변수/상수
#define pi      3.141592
#define DTR(x)  (x)*(pi/180) // degree to radian
#define RTD(x)  (x)*(180/pi) // radian to degree

const float d = 79.75; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 109.21; // 2번과 3번 모터의 회전축간 거리
const float L2 = 86.4; // 3번과 4번 모터의 회전축간 거리
const float L3 = 97.2; // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리

// 숙였을 때 카메라에 바퀴가 안보이는 정도
#define MANIPULATOR_M1_UPPER_LIMIT       20 // 우측 극대값
#define MANIPULATOR_M1_DEFAULT            0
#define MANIPULATOR_M1_LOWER_LIMIT      -20 // 좌측 극대값

#define MANIPULATOR_M2_DEFAULT          -15 // 곧바로 서는 각

#define MANIPULATOR_M3_DEFAULT          -130

#define MANIPULATOR_M4_UPPER_LIMIT      50 // 몸체에 그리퍼가 닿지 않을 정도
#define MANIPULATOR_M4_DEFAULT          0
#define MANIPULATOR_M4_LOWER_LIMIT      -40

#define MOBILEBASE_DRIVING_SPEED_LIMIT  100
#define MOBILEBASE_ROTATION_LIMIT       200

#define PIXY2_X_SETPOINT                157   // 카메라 x위치 목표값
#define PIXY2_Y_SETPOINT                103   // 카메라 y위치 목표값
#define DAMPING_FACTOR                   15   // 카메라 인식 물체 추적시
                                              // 댐핑 팩터(클수록 조금씩 이동)
#define PIXY2_DEFAULT_AREA              800   // 카메라 인식 물체의 기본 설정 면적        

float manipulatorM1Angle = MANIPULATOR_M1_DEFAULT;
float manipulatorM2Angle = MANIPULATOR_M2_DEFAULT;
float manipulatorM3Angle = MANIPULATOR_M3_DEFAULT;
float manipulatorM4Angle = MANIPULATOR_M4_DEFAULT;

int16_t mobilebaseRotation = 0; // 좌/우회전 결정, 양쪽 바퀴 속도차
int16_t mobilebaseDrivingSpeed = 0; // 주행 속도


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  pixy.init(); // pixy객체 초기화
  
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  motorInitialize();
}

void loop()
{
  pixy.ccc.getBlocks();
                        
  if( pixy.ccc.numBlocks )
  {
    if( pixy.ccc.blocks[0].m_signature == 1 ) // 시그니처 1번으로 인식되면
    {
      // 인식된 물체의 요소별 기준으로부터의 변화량 계산
      int16_t dX = pixy.ccc.blocks[0].m_x - PIXY2_X_SETPOINT;
      int16_t dY = pixy.ccc.blocks[0].m_y - PIXY2_Y_SETPOINT;

      // 카메라의 dX로 매니퓰레이터의 수평회전 결정
      int16_t nextManM1Angle = manipulatorM1Angle + dX/DAMPING_FACTOR;
      
      manipulatorM1Angle = constrain( nextManM1Angle,
                                      MANIPULATOR_M1_LOWER_LIMIT,
                                      MANIPULATOR_M1_UPPER_LIMIT );

      // 카메라의 dY로 매니퓰레이터의 높이 결정
      int16_t nextManM4Angle = manipulatorM4Angle - dY/DAMPING_FACTOR;
      
      manipulatorM4Angle = constrain( nextManM4Angle,
                                      MANIPULATOR_M4_LOWER_LIMIT,
                                      MANIPULATOR_M4_UPPER_LIMIT );

      // 매니퓰레이터 제어
      SetManipulatorForwardMoveForSyncWrite( manipulatorM1Angle,
                                             manipulatorM2Angle,
                                             manipulatorM3Angle,
                                             manipulatorM4Angle, 200);
      while(dxl.syncWrite(&sw_arm_infos) != true){}
                            
      // 매니퓰레이터 수평 회전 변화량, 물체 크기 변화량 계산
      int16_t dR = manipulatorM1Angle - MANIPULATOR_M1_DEFAULT;
      uint16_t objectArea = (pixy.ccc.blocks[0].m_width
                             * pixy.ccc.blocks[0].m_height); // 물체 인식 면적
      int32_t dD = (int32_t)objectArea - PIXY2_DEFAULT_AREA;  // 면적 변화량
      
      // 매니퓰레이터 회전 변화량으로 모바일베이스의 회전속도 결정. 양쪽 바퀴의 속도차를 만듦
      int16_t nextMobRotation = dR * 7;
      mobilebaseRotation = constrain( nextMobRotation,
                                      -MOBILEBASE_ROTATION_LIMIT,
                                      MOBILEBASE_ROTATION_LIMIT );

      // dD가 음수일 때는 물체가 멀리 있기 때문에 로봇이 전진하도록 적당한 값의 음수를 곱함
      // 양수일 때는 물체가 가까이 있기 때문에 로봇이 후진하도록 적당한 값의 양수를 곱함
      int16_t nextMobDrivingSpeed = dD * -0.1;  
      
      
      mobilebaseDrivingSpeed = constrain( nextMobDrivingSpeed,
                                          -MOBILEBASE_DRIVING_SPEED_LIMIT,
                                          MOBILEBASE_DRIVING_SPEED_LIMIT );

      // 모바일베이스 제어
      SetMobileGoalVelocityForSyncWrite( mobilebaseDrivingSpeed
                                         + mobilebaseRotation/2,
                                         mobilebaseDrivingSpeed
                                         - mobilebaseRotation/2,
                                         mobilebaseDrivingSpeed
                                         + mobilebaseRotation/2,
                                         mobilebaseDrivingSpeed
                                         - mobilebaseRotation/2 );
      while(dxl.syncWrite(&sw_mobile_infos) != true){}
      delay(200);
    } else {  // 카메라 뷰에 검출된 객체가 없는 경우: 정지
      // 모바일베이스 제어
      SetMobileGoalVelocityForSyncWrite( 0, 0, 0, 0 );
      while(dxl.syncWrite(&sw_mobile_infos) != true){}
    }
  } else {  // 카메라 뷰에 검출된 객체가 없는 경우: 정지
      // 모바일베이스 제어
      SetMobileGoalVelocityForSyncWrite( 0, 0, 0, 0 );
      while(dxl.syncWrite(&sw_mobile_infos) != true){}
  }
}


// 모터 초기화
void motorInitialize() {
    // 매니퓰레이터 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindManipulatorServos()) {}
  
  // 각 모터에 설정
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 0);
  
    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Time-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i],
                              REVERSE_MODE + TIME_BASED_PROFILE);
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], 3);

    // 각 모터의 최소/최대/offset 값 설정
    dxl.writeControlTableItem(MIN_POSITION_LIMIT, ARM_DXL_IDS[i],
                              ARM_DXL_POSITION_MIN[i]);
    dxl.writeControlTableItem(MAX_POSITION_LIMIT, ARM_DXL_IDS[i],
                              ARM_DXL_POSITION_MAX[i]);
    dxl.writeControlTableItem(HOMING_OFFSET, ARM_DXL_IDS[i],
                              ARM_DXL_OFFSET[i]);

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 1);
  }
  
  // manipulator sync write 준비
  sw_arm_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_arm_infos.packet.is_completed = false; // false로 초기화
  sw_arm_infos.addr = SW_JOINT_START_ADDR; // 컨트롤 테이블에 sync write 시작주소
  sw_arm_infos.addr_length = SW_JOINT_DATA_SIZE; // sync write하는 데이터길이
  sw_arm_infos.p_xels = info_arm_xels_sw; // sync write 할 모터 정보
  sw_arm_infos.xel_count = 0; // sync write 할 모터 개수

  sw_joint_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  sw_joint_data[0].goal_position = 2048;
  sw_joint_data[1].profile_velocity = 1000;
  sw_joint_data[1].goal_position = 2048;
  sw_joint_data[2].profile_velocity = 1000;
  sw_joint_data[2].goal_position = 2048;
  sw_joint_data[3].profile_velocity = 1000;
  sw_joint_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_arm_xels_sw[i].id = ARM_DXL_IDS[i];
    info_arm_xels_sw[i].p_data = (uint8_t*)&sw_joint_data[i];
    sw_arm_infos.xel_count++;
  }
  sw_arm_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정

  SetManipulatorForwardMoveForSyncWrite( manipulatorM1Angle,
                                         manipulatorM2Angle,
                                         manipulatorM3Angle,
                                         manipulatorM4Angle, 2000 ); 
  while(!dxl.syncWrite(&sw_arm_infos)) {}

  
  // 모바일베이스 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindMobileBaseServos()) {}
  
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
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
  
  // mobilebase sync write 준비
  sw_mobile_infos.packet.p_buf = nullptr;
  sw_mobile_infos.packet.is_completed = false;
  sw_mobile_infos.addr = SW_WHEEL_START_ADDR;
  sw_mobile_infos.addr_length = SW_WHEEL_DATA_SIZE;
  sw_mobile_infos.p_xels = info_mobile_xels_sw;
  sw_mobile_infos.xel_count = 0;

  sw_wheel_data[0].goal_velocity = 0; // 모터에 write 할 데이터 초기화
  sw_wheel_data[1].goal_velocity = 0;
  sw_wheel_data[2].goal_velocity = 0;
  sw_wheel_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_mobile_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_mobile_xels_sw[i].p_data = (uint8_t*)&sw_wheel_data[i].goal_velocity;
    sw_mobile_infos.xel_count++;
  }
  sw_mobile_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정

  SetMobileGoalVelocityForSyncWrite( 0, 0, 0, 0 );  // 정지
  while(!dxl.syncWrite(&sw_mobile_infos)) {}
  
  delay(3000);
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
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고
    // 0~4095 범위로 매핑
    return (int32_t)map(constrain((int32_t)angle, -180, 180),
                        -180, 180, 0, 4095);
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
    sw_joint_data[0].goal_position = motor1GoalPosition;
    sw_joint_data[0].profile_velocity = operatingTime;
  }
  if (motor2GoalPosition != -1) {
    sw_joint_data[1].goal_position = motor2GoalPosition;
    sw_joint_data[1].profile_velocity = operatingTime;
  }
  if (motor3GoalPosition != -1) {
    sw_joint_data[2].goal_position = motor3GoalPosition;
    sw_joint_data[2].profile_velocity = operatingTime;
  }
  if (motor4GoalPosition != -1) {
    sw_joint_data[3].goal_position = motor4GoalPosition;
    sw_joint_data[3].profile_velocity = operatingTime;
  }
  
  sw_arm_infos.is_info_changed = true;
}

/*
 * MOBILE_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수. 모두 통신 가능하면 true, 아니면 false를 반환
 */
bool FindMobileBaseServos() {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= sizeof(MOBILE_DXL_IDS)/sizeof(uint8_t)) {
      uint8_t mobile_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (MOBILE_DXL_IDS[mobile_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          mobile_dxl_ids_idx ++;

          if (mobile_dxl_ids_idx
              == sizeof(MOBILE_DXL_IDS)/sizeof(uint8_t)) {
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
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 */
void SetMobileGoalVelocityForSyncWrite(int32_t fl_goal_velocity,
                                       int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity,
                                       int32_t br_goal_velocity) {
  sw_wheel_data[0].goal_velocity = fl_goal_velocity;
  sw_wheel_data[1].goal_velocity = fr_goal_velocity;
  sw_wheel_data[2].goal_velocity = bl_goal_velocity;
  sw_wheel_data[3].goal_velocity = br_goal_velocity;
  sw_mobile_infos.is_info_changed = true;
}
