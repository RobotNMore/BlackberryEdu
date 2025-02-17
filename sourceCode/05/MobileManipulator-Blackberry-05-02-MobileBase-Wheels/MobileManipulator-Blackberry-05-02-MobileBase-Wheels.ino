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
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = {MOBILE_DXL_ID_FL,
                                                   MOBILE_DXL_ID_FR,
                                                   MOBILE_DXL_ID_BL,
                                                   MOBILE_DXL_ID_BR};

// sync write 용 상수, 구조체 정의 및 인스턴스화
const int32_t GOAL_VELOCITY_VALUE = 88; // 모터에 적용할 속도 값
const uint16_t GOAL_VELOCITY_ADDR = 104; // sync write start 주소
const uint16_t GOAL_VELOCITY_SIZE = 4; // sync write data 길이
typedef struct sw_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;

sw_data_t sw_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[MOBILE_DXL_ID_CNT];


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

  // sync write 준비
  sw_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_infos.packet.is_completed = false; // false로 초기화
  sw_infos.addr = GOAL_VELOCITY_ADDR; // 컨트롤 테이블에 sync write를 시작하는 주소
  sw_infos.addr_length = GOAL_VELOCITY_SIZE; // sync write하는 데이터 길이
  sw_infos.p_xels = info_xels_sw; // sync write 할 모터 정보
  sw_infos.xel_count = 0; // sync write 할 모터 개수

  sw_data[0].goal_velocity = 0; // 모터에 write 할 데이터 초기화
  sw_data[1].goal_velocity = 0;
  sw_data[2].goal_velocity = 0;
  sw_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정 
}

void loop()
{
  SetMobileGoalVelocityForSyncWrite(GOAL_VELOCITY_VALUE, GOAL_VELOCITY_VALUE,
                                    0, 0);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
  delay(2000);
  
  
  SetMobileGoalVelocityForSyncWrite(-GOAL_VELOCITY_VALUE, -GOAL_VELOCITY_VALUE,
                                    0, 0);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
  delay(2000);
  
  
  SetMobileGoalVelocityForSyncWrite(0, 0, 0, 0);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
  delay(2000);
  
  
  SetMobileGoalVelocityForSyncWrite(0, 0,
                                    GOAL_VELOCITY_VALUE, GOAL_VELOCITY_VALUE);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
  delay(2000);
  
  SetMobileGoalVelocityForSyncWrite(0, 0,
                                    -GOAL_VELOCITY_VALUE, -GOAL_VELOCITY_VALUE);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
  delay(2000);
  
  SetMobileGoalVelocityForSyncWrite(0, 0, 0, 0);
  while(dxl.syncWrite(&sw_infos) == false) { delay(10); }
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

/*
 * sync write 하기 위한 모바일베이스의 goal velocity 값들을 설정하는 함수
 */
void SetMobileGoalVelocityForSyncWrite(int32_t fl_goal_velocity,
                                       int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity,
                                       int32_t br_goal_velocity) {
  sw_data[0].goal_velocity = fl_goal_velocity;
  sw_data[1].goal_velocity = fr_goal_velocity;
  sw_data[2].goal_velocity = bl_goal_velocity;
  sw_data[3].goal_velocity = br_goal_velocity;
  sw_infos.is_info_changed = true;
}
