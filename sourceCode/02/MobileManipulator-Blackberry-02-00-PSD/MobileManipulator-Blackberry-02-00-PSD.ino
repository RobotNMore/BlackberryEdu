void setup() {
  Serial.begin( 9600 ); // 시리얼 통신속도 (baudrate)
}

void loop() {
  int left = analogRead( A0 );  // 전방 좌측 PSD센서

  Serial.println( left );
  delay(500);
}
