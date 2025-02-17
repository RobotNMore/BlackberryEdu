//////////////  모터 구동 관련 선언
//#define CHANGE_MOTOR_BAUDRATE_57600_TO_1M
#define CHANGE_MOTOR_BAUDRATE_1M_TO_57600

#define MOTOR_BAUDRATE_57600    (int32_t)1
#define MOTOR_BAUDRATE_1M       (int32_t)3

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

const uint8_t DXL_CNT = 8;
const uint8_t DXL_IDS[DXL_CNT] = {1, 2, 3, 4, 5, 6, 7, 8};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  메인 프로그램

void setup() {
  Serial.begin(115200);

#if (defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M) && defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600)) || (!defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M) && !defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600))
  Serial.println("Only one CHANGE_MOTOR_BAUDRATE must be defined.");
  while(1) { delay(1000); }
#endif

#if defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M)
  dxl.begin((int32_t)57600);
#elif defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600)
  dxl.begin((int32_t)1000000);
#endif
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindServos()) {}

  Serial.println("start setting");
  
  // 각 모터에 설정
  for (int i = 0 ; i < DXL_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, DXL_IDS[i], 0);
    
    // baudrate 설정
#if defined(CHANGE_MOTOR_BAUDRATE_57600_TO_1M)
    dxl.writeControlTableItem(BAUD_RATE, DXL_IDS[i], MOTOR_BAUDRATE_1M);
#elif defined(CHANGE_MOTOR_BAUDRATE_1M_TO_57600)
    dxl.writeControlTableItem(BAUD_RATE, DXL_IDS[i], MOTOR_BAUDRATE_57600);
#endif
  }
  
  Serial.println("complete setting");
}

void loop() {}


/*
 * DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수. 모두 통신 가능하면 true, 아니면 false를 반환
 */
bool FindServos() {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= DXL_CNT) {
      uint8_t dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (DXL_IDS[dxl_ids_idx]
            == ids_pinged[dxl_ids_idx++]) {
          dxl_ids_idx ++;

          if (dxl_ids_idx == DXL_CNT) {
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
