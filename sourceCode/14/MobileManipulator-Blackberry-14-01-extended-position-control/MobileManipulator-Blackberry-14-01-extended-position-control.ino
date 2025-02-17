//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2; // DIR PIN
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  모터 구동용 변수/상수
#define TORQUE_ON                 0x01 // TORQUE_ENABLE 관련
#define TORQUE_OFF                0x00

#define NORMAL_MODE                 0x00 // drive mode의 방향 모드
#define REVERSE_MODE                0x01

#define VELOCITY_BASED_PROFILE      0x00 // profile configuration
#define TIME_BASED_PROFILE          0x04

#define VELOCITY_CONTROL_MODE               0x01 // OPERATING_MODE 관련
#define POSITION_CONTROL_MODE               0x03
#define EXTENDED_POSITION_CONTROL_MODE      0x04

#define MOBILE_DXL_ID_FL        0x01 // 좌측 전방 모터 아이디
#define MOBILE_DXL_ID_FR        0x02 // 우측 전방 모터 아이디
#define MOBILE_DXL_ID_BL        0x03 // 좌측 후방 모터 아이디
#define MOBILE_DXL_ID_BR        0x04 // 우측 후방 모터 아이디

// 4095(:모터 해상도)/2*PI(:1회전 각도 [rad])
#define RADIANS_2_MOTOR_VALUE_RATIO (4095/(2*PI))
#define RADIANS_2_MOTOR_VALUE(X)    (int32_t)((X)*RADIANS_2_MOTOR_VALUE_RATIO)

#define WHEEL_DIAMETER_MM           60.0 // 빨간바퀴 지름
#define WHEEL_RADIUS_MM             (WHEEL_DIAMETER_MM/2.0)

#define S_TO_MILLIS_RATIO                       1000

// FindMobileBaseServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t MOBILE_DXL_ID_CNT = 4;
const uint8_t MOBILE_DXL_IDS[MOBILE_DXL_ID_CNT] = {MOBILE_DXL_ID_FL,
                                                   MOBILE_DXL_ID_FR,
                                                   MOBILE_DXL_ID_BL,
                                                   MOBILE_DXL_ID_BR};


// mobile extended position control mode sync write 용
const uint16_t SW_WHEEL_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_WHEEL_POSITION_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_wheel_position_data { // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_wheel_position_data_t;

sw_wheel_position_data_t sw_wheel_position_data[MOBILE_DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_position_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_wheel_position_xels_sw[MOBILE_DXL_ID_CNT];


// mobile extended position control mode sync read 용
const uint16_t wheel_wheel_position_pkt_buf_cap = 128;
uint8_t wheel_position_pkt_buf[wheel_wheel_position_pkt_buf_cap];

const uint16_t SR_WHEEL_POSITION_START_ADDR = 132; // sync read start 주소
const uint16_t SR_WHEEL_POSITION_DATA_SIZE = 4; // sync read data 길이
typedef struct sr_wheel_position_data{ // 모터에서 데이터를 읽기 위한 구조체 정의
  int32_t present_position;
} __attribute__((packed)) sr_wheel_position_data_t;

sr_wheel_position_data_t sr_wheel_position_data[MOBILE_DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_mobile_position_infos;
DYNAMIXEL::XELInfoSyncRead_t info_wheel_position_xels_sr[MOBILE_DXL_ID_CNT];


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  // 모바일베이스 모터가 모두 있는지 확인될 때 까지 진행하지 않음
  while(!FindMobileBaseServos()) {}
  
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);
  
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i],
                                NORMAL_MODE + TIME_BASED_PROFILE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i],
                                REVERSE_MODE + TIME_BASED_PROFILE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i],
                              EXTENDED_POSITION_CONTROL_MODE);

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
  
  // mobilebase velocity extended position control mode sync write 준비
  sw_mobile_position_infos.packet.p_buf = nullptr;
  sw_mobile_position_infos.packet.is_completed = false;
  sw_mobile_position_infos.addr = SW_WHEEL_POSITION_START_ADDR;
  sw_mobile_position_infos.addr_length = SW_WHEEL_POSITION_DATA_SIZE;
  sw_mobile_position_infos.p_xels = info_wheel_position_xels_sw;
  sw_mobile_position_infos.xel_count = 0;

   // 모터에 write 할 데이터 초기화. profile_velocity는 적당한 값 아무거나
  sw_wheel_position_data[0].profile_velocity = 10000;
  sw_wheel_position_data[0].goal_position = dxl.readControlTableItem(PRESENT_POSITION,
                                                                     MOBILE_DXL_IDS[0]);
  sw_wheel_position_data[1].profile_velocity = 10000;
  sw_wheel_position_data[1].goal_position = dxl.readControlTableItem(PRESENT_POSITION,
                                                                     MOBILE_DXL_IDS[1]);
  sw_wheel_position_data[2].profile_velocity = 10000;
  sw_wheel_position_data[2].goal_position = dxl.readControlTableItem(PRESENT_POSITION,
                                                                     MOBILE_DXL_IDS[2]);
  sw_wheel_position_data[3].profile_velocity = 10000;
  sw_wheel_position_data[3].goal_position = dxl.readControlTableItem(PRESENT_POSITION,
                                                                     MOBILE_DXL_IDS[3]);

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sw[i].p_data = (uint8_t*)&sw_wheel_position_data[i];
    sw_mobile_position_infos.xel_count++;
  }
  sw_mobile_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // mobilebase extended position control mode sync read 준비
  sr_mobile_position_infos.packet.buf_capacity = wheel_wheel_position_pkt_buf_cap;
  sr_mobile_position_infos.packet.p_buf = wheel_position_pkt_buf;
  sr_mobile_position_infos.packet.is_completed = false;
  sr_mobile_position_infos.addr = SR_WHEEL_POSITION_START_ADDR;
  sr_mobile_position_infos.addr_length = SR_WHEEL_POSITION_DATA_SIZE;
  sr_mobile_position_infos.p_xels = info_wheel_position_xels_sr;
  sr_mobile_position_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sr[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sr[i].p_recv_buf = (uint8_t*)&sr_wheel_position_data[i];
    sr_mobile_position_infos.xel_count++;
  }
  sr_mobile_position_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  
  delay(5000);

  DriveXYDistanceAndMmPerSec(100, 100, 72);  // 대각선으로 141.4mm를 약 2초동안 주행 후 정지
  while(!dxl.syncWrite(&sw_mobile_position_infos)) {}
}

void loop() {}


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
 * 멀티턴모드, 시간기반 프로파일 사용하여 모바일베이스를 정해진 X,Y 거리만큼 이동시키는 함수
 * params :
 *    xMm : 수평으로 이동할 거리(mm 단위), 우진할 때 +값
 *    yMm : 수직으로 이동할 거리(mm 단위), 전진할 때 +값
 *    mmPerSec : 1초에 몇 mm만큼 주행할 것인지 속도
 */
void DriveXYDistanceAndMmPerSec(float xMm, float yMm, int32_t mmPerSec) {
  float distance = sqrt(xMm*xMm + yMm*yMm);
  
  float motor1ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm + xMm)/WHEEL_RADIUS_MM);
  float motor2ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm - xMm)/WHEEL_RADIUS_MM);
  float motor3ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm - xMm)/WHEEL_RADIUS_MM);
  float motor4ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm + xMm)/WHEEL_RADIUS_MM);

  SetMobileRelativePositionForSyncWrite(motor1ValueForRotation,
                                        motor2ValueForRotation,
                                        motor3ValueForRotation,
                                        motor4ValueForRotation,
                                        (distance/mmPerSec)*S_TO_MILLIS_RATIO);
}

/*
 * 멀티턴모드를 사용하여 정해진 바퀴를 모터 값 만큼 회전시키는 함수
 * params :
 *    fl_relative_position : 좌측전방 바퀴 모터 회전 값 
 *    fr_relative_position : 우측전방 바퀴 모터 회전 값
 *    bl_relative_position : 좌측후방 바퀴 모터 회전 값
 *    br_relative_position : 우측전방 바퀴 모터 회전 값
 *    drivingTime : 이동에 사용할 시간
 */
void SetMobileRelativePositionForSyncWrite(int32_t fl_relative_position,
                                           int32_t fr_relative_position,
                                           int32_t bl_relative_position,
                                           int32_t br_relative_position,
                                           int32_t drivingTime) {
  uint8_t recv_cnt = 0;
  while(1) {
    recv_cnt = dxl.syncRead(&sr_mobile_position_infos);
    if(recv_cnt == MOBILE_DXL_ID_CNT){
      break;
    } else {
      Serial.print("[syncRead] Fail, Lib error code: ");
      Serial.println(dxl.getLastLibErrCode());
    }
  }

  // 제어 가능한 최대값을 넘지 않도록 확인
  if (abs(sr_wheel_position_data[0].present_position + fl_relative_position) < 1048575
      && abs(sr_wheel_position_data[1].present_position + fr_relative_position) < 1048575
      && abs(sr_wheel_position_data[2].present_position + bl_relative_position) < 1048575
      && abs(sr_wheel_position_data[3].present_position + br_relative_position) < 1048575) {
    sw_wheel_position_data[0].profile_velocity = drivingTime;
    sw_wheel_position_data[0].goal_position = sr_wheel_position_data[0].present_position
                                              + fl_relative_position;
    sw_wheel_position_data[1].profile_velocity = drivingTime;
    sw_wheel_position_data[1].goal_position = sr_wheel_position_data[1].present_position
                                              + fr_relative_position;
    sw_wheel_position_data[2].profile_velocity = drivingTime;
    sw_wheel_position_data[2].goal_position = sr_wheel_position_data[2].present_position
                                              + bl_relative_position;
    sw_wheel_position_data[3].profile_velocity = drivingTime;
    sw_wheel_position_data[3].goal_position = sr_wheel_position_data[3].present_position
                                              + br_relative_position;
                                              
    sw_mobile_position_infos.is_info_changed = true;
  }
}
