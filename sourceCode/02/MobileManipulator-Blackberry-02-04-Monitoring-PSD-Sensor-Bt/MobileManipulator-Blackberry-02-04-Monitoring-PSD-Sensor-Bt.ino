//////////////  전방 PSD센서

#define PIN_FRONT_LEFT_PSD    A0 // 매니퓰레이터 전방 좌측 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 매니퓰레이터 전방 우측 PSD센서 연결 핀

//////////////  메인 프로그램

void setup() {
  Serial2.begin( 115200 ); // 블루투스 시리얼 초기화 (baudrate)
}

void loop() {
  int frontLeftPSDValue = analogRead( PIN_FRONT_LEFT_PSD );  // PSD left 값 읽기
  int frontRightPSDValue = analogRead( PIN_FRONT_RIGHT_PSD ); // PSD right 값 읽기

  Serial2.print( "front left : " ); // 시리얼 모니터에 "front left : "를 출력
  Serial2.print( frontLeftPSDValue ); // frontLeftPSDValue변수의 값을 출력
  Serial2.print( ", front right : " );
  Serial2.print( frontRightPSDValue );
  Serial2.println(); // 시리얼 모니터 개행
  delay(100);
}
