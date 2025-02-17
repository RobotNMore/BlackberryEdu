//////////////  메인 프로그램

void setup() {
  Serial.begin( 9600 ); // 시리얼 통신 (baudrate)
}

void loop() {
  unsigned long currentTime = millis(); // 아두이노 부팅 후 지금까지
                                        // 경과한 시간(ms)을 변수에 저장
  
  Serial.print( "Hello" ); // 시리얼 모니터에 "Hello"를 출력
  Serial.print( ", " );    // 앞서 출력한 Hello에 이어 붙여서 ", "를 출력
  Serial.println( "world" ); // 앞서 출력한 Hello, 에 이어붙여서 world를
                             // 출력 후 개행(NewLine)
  Serial.println( "Hello, arduino" ); // 시리얼 모니터에 출력 후 개행
  Serial.println(); // 개행
  delay(3000); // 3초 대기
}
