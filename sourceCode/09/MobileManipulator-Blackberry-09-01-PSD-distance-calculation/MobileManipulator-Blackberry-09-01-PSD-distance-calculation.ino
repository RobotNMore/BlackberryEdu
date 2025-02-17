//////////////  전방 PSD센서

#define PIN_FRONT_LEFT_PSD    A0 // 매니퓰레이터 전방 좌측 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 매니퓰레이터 전방 우측 PSD센서 연결 핀
 
//////////////  메인 프로그램

void setup() {
  Serial.begin( 115200 );
}

void loop() {
  int frontLeftPSDValue = analogRead( PIN_FRONT_LEFT_PSD );  // PSD left 값 읽기
  int frontRightPSDValue = analogRead( PIN_FRONT_RIGHT_PSD ); // PSD right 값 읽기

  //float leftDistance = 1/((frontLeftPSDValue*2.059e-4) - 1.473e-3);
  //float rightDistance = 1/((frontRightPSDValue*2.773e-4) - 3.84e-2);

  float leftDistance = 13768 * pow(frontLeftPSDValue, -1.183);   // 전방 좌측 거리 추세 함수
  float rightDistance = 15911 * pow(frontRightPSDValue, -1.2);     // 전방 우측 거리 추세 함수

  Serial.print( "left : " ); // 시리얼 모니터에 "left : "를 출력
  Serial.print( leftDistance ); // frontLeftPSDValue변수의 값을 출력
  Serial.print( " cm, right : " );
  Serial.print( rightDistance );
  Serial.print( " cm" );
  Serial.println(); // 시리얼 모니터 개행
  delay(100);
}
