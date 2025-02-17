//////////////  모터 구동 관련 선언
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
const int DXL_DIR_PIN = 2; // DIR PIN

const uint8_t DXL_ID = 7;
const float DXL_PROTOCOL_VERSION = 2.0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 컨트롤 테이블 아이템의 이름을 사용하기 위해 이 네임스페이스가 필요함
using namespace ControlTableItem;


//////////////  메인 프로그램

void setup() {
  Serial.begin(115200);

  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}

void loop() {
  Serial.print("PROTOCOL ");
  Serial.print(DXL_PROTOCOL_VERSION, 1);
  Serial.print(", ID ");
  Serial.print(DXL_ID);
  Serial.print(": ");
  
  if(dxl.ping(DXL_ID) == true){  // 핑 체크 성공
    Serial.print("ping succeeded!");  // 모델 넘버를 출력
    Serial.print(", Model Number: ");
    Serial.println(dxl.getModelNumber(DXL_ID));
    
    if (dxl.readControlTableItem(LED, DXL_ID) == 1) {  // 현재 LED 상태가 On이면
      dxl.writeControlTableItem(LED, DXL_ID, 0);  // LED 끄기
    } else {  // 현재 LED 상태가 Off이면
      dxl.writeControlTableItem(LED, DXL_ID, 1);  // LED 켜기
    }
  }else{  // 핑 체크 실패
    Serial.print("ping failed!, err code: ");
    Serial.println(dxl.getLastLibErrCode());  // 에러코드 출력
  }
  delay(1000);

  FindServos();  // 통신 가능한 모든 모터의 정보를 출력하는 함수
  delay(5000);
}


/*
 * 통신 가능한 모든 모터를 찾아 모델 넘버와 펌웨어 버전을 출력하는 함수
 */
DYNAMIXEL::InfoFromPing_t ping_info[32];

void FindServos(void) {
  Serial.println("  Try Protocol 2 - broadcast ping");
  Serial.flush(); // flush it as ping may take a while... 
      
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ping_info, 
    sizeof(ping_info)/sizeof(ping_info[0]), 100)) {
      
    Serial.print("pinged count : ");
    Serial.println(count_pinged);
    
    Serial.print("Detected Dynamixel : \n");
    for (int i = 0; i < count_pinged; i++)
    {
      Serial.print("    ");
      Serial.print(ping_info[i].id, DEC);
      Serial.print(", Model:");
      Serial.print(ping_info[i].model_number);
      Serial.print(", Ver:");
      Serial.println(ping_info[i].firmware_version, DEC);
    }
  } else{
    Serial.print("Broadcast returned no items : ");
    Serial.println(dxl.getLastLibErrCode());
  }
}

/*
 * 
 * [참고] 만약 보드의 모터 스위치를 켜도 계속해서 다음과 같이
 *
 *             Try Protocol 2 - broadcast ping: 
 *          Broadcast returned no items : 3
 *          PROTOCOL 2.0, ID 7: ping failed!, err code: 3
 *
 *       메시지만 나온다면 현재 모터의 통신 속도가 다른 속도로 설정되어 있을 수 있습니다.
 * 
 *       ===> [settingTools] 폴더의 changeMotorBaudrate.ino 프로그램으로 변경 가능
 *       
 *       (통신이 정상인 경우의 메시지 샘플)
 *       
 *            Try Protocol 2 - broadcast ping: 
 *          pinged count : 8
 *          Detected Dynamixel : 
 *              1, Model:1060, Ver:46
 *              2, Model:1060, Ver:46
 *              3, Model:1060, Ver:46
 *              4, Model:1060, Ver:46
 *              5, Model:1060, Ver:46
 *              6, Model:1060, Ver:46
 *              7, Model:1060, Ver:46
 *              8, Model:1060, Ver:46
 *          PROTOCOL 2.0, ID 7: ping succeeded!, Model Number: 1060
 */
