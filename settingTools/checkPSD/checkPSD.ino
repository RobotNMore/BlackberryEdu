//////////////  전방 PSD센서

#define PIN_FRONT_LEFT_PSD    A0 // 전방 좌측 PSD센서 연결 핀
#define PIN_SIDE_LEFT_PSD     A1 // 좌측면 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 전방 우측 PSD센서 연결 
#define PIN_SIDE_RIGHT_PSD    A3 // 우측면 PSD센서 연결 핀

 
//////////////  메인 프로그램

void setup() {
  Serial.begin( 115200 ); // 블루투스 시리얼 초기화 (baudrate)
}

void loop() {
  int frontLeftPSDValue = analogRead( PIN_FRONT_LEFT_PSD );  // PSD front left 값 읽기
  int frontRightPSDValue = analogRead( PIN_FRONT_RIGHT_PSD ); // PSD front right 값 읽기
  int sideLeftPSDValue = analogRead( PIN_SIDE_LEFT_PSD );  // PSD side left 값 읽기
  int sideRightPSDValue = analogRead( PIN_SIDE_RIGHT_PSD ); // PSD side right 값 읽기

  Serial.print( "sl : " );
  Serial.print( sideLeftPSDValue );
  Serial.print( ", fl : " ); // 시리얼 모니터에 "fl : "를 출력
  Serial.print( frontLeftPSDValue ); // frontLeftPSDValue변수의 값을 출력
  Serial.print( ", fr : " );
  Serial.print( frontRightPSDValue );
  Serial.print( ", sr : " );
  Serial.print( sideRightPSDValue );
  Serial.println(); // 시리얼 모니터 개행
  delay(100);
}
