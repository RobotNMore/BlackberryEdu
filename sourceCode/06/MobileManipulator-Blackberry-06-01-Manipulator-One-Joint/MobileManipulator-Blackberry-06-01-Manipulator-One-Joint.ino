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
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = {ARM_DXL_ID_1,
                                             ARM_DXL_ID_2,
                                             ARM_DXL_ID_3,
                                             ARM_DXL_ID_4
                                            };


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while (!FindManipulatorServos()) {}

  // 각 모터에 설정
  for (int i = 0 ; i < sizeof(ARM_DXL_IDS) / sizeof(uint8_t) ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 0);

    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Velocity-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i], REVERSE_MODE);
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], 3);

    switch (ARM_DXL_IDS[i]) {
      case ARM_DXL_ID_1:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_1_OFFSET);
        break;
      case ARM_DXL_ID_2:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_2_OFFSET);
        break;
      case ARM_DXL_ID_3:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_3_OFFSET);
        break;
      case ARM_DXL_ID_4:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_4_OFFSET);
        break;
    }

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], 1);

    // 위치 제어 모드에서는 Goal velocity가 아닌 profile velocity를 사용
    // 약 10 rpm으로 목표속도 설정
    dxl.writeControlTableItem(PROFILE_VELOCITY, ARM_DXL_IDS[i], 44);
  }
}

void loop()
{
  MoveArmServoWithAngle(ARM_DXL_ID_1, -90);
  delay(2000);

  MoveArmServoWithAngle(ARM_DXL_ID_1, 0);
  delay(2000);

  MoveArmServoWithAngle(ARM_DXL_ID_1, 90);
  delay(2000);

  MoveArmServoWithAngle(ARM_DXL_ID_1, 0);
  delay(5000);
}

/*
   ARM_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
   확인하는 함수. 모두 통신 가능하면 true, 아니면 false를 반환
*/
bool FindManipulatorServos() {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged,
                                      sizeof(ids_pinged) / sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= sizeof(ARM_DXL_IDS) / sizeof(uint8_t)) {
      uint8_t arm_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while (1) {
        if (ARM_DXL_IDS[arm_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          arm_dxl_ids_idx ++;

          if (arm_dxl_ids_idx
              == sizeof(ARM_DXL_IDS) / sizeof(uint8_t)) {
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
  } else {
    Serial.print("Broadcast returned no items : ");
    Serial.println(dxl.getLastLibErrCode());
    is_each_motor_found = false;
  }
  return is_each_motor_found;
}

/*
   모터의 각도를 제어하는 함수
   params :
      motorID : 제어할 모터의 아이디
      angle : 제어할 모터의 목표 각도
*/
void MoveArmServoWithAngle(uint8_t motorID, float angle) {
  if (!isnan(angle)) {
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고
    // 0~4095 범위로 매핑
    int32_t motorPosition = map(constrain((int32_t)angle, -180, 180),
                                -180, 180, 0, 4095);
    dxl.writeControlTableItem(GOAL_POSITION, motorID, motorPosition);
  }
}
