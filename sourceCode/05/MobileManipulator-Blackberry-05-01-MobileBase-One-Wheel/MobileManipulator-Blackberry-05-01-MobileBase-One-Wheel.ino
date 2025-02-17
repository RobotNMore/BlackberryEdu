//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  모터 구동용 변수/상수
#define MOBILE_DXL_ID_FL            0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR            0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL            0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR            0x04 // 우측 후방 모터 아이디

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_IDS[4] = {MOBILE_DXL_ID_FL, MOBILE_DXL_ID_FR,
                                   MOBILE_DXL_ID_BL, MOBILE_DXL_ID_BR};


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindMobileBaseServos()) {}

  // 각 모터에 설정
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
}

void loop()
{                    // 전방 좌측 모터를 제어
                     // 모터 회전 속도(양수는 CCW(전진), 음수는 CW(후진))
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FL, 44);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FL, -44);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FL, 88);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FL, -88);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FL, 0);
  delay(5000);


  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FR, 44);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FR, -44);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FR, 88);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FR, -88);
  delay(2000);
  
  dxl.writeControlTableItem(GOAL_VELOCITY, MOBILE_DXL_ID_FR, 0);
  delay(5000);
  
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
