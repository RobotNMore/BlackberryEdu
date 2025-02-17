void setup() {
  pinMode(4, OUTPUT); // LED 핀을 출력으로 설정
}

void loop() {
  digitalWrite(4, HIGH); // 유저 LED 켜기
  delay(1000);
  
  digitalWrite(4, LOW);  // 유저 LED 끄기
  delay(1000);
}
