//////////////  사용자 RED LED
#define RED_LED_PIN 4

void setup() {
  pinMode(RED_LED_PIN, OUTPUT); // LED 핀을 출력으로 설정
}

void loop() {
  digitalWrite(RED_LED_PIN, HIGH); // 유저 LED 켜기
  delay(1000); // 1초 유지
  digitalWrite(RED_LED_PIN, LOW); // 유저 LED 끄기
  delay(1000); // 1초 기다리기
  
  digitalWrite(RED_LED_PIN, HIGH); // 유저 LED 켜기
  delay(500); // 0.5초 유지
  digitalWrite(RED_LED_PIN, LOW); // 유저 LED 끄기
  delay(1000); // 1초 기다리기
  
  digitalWrite(RED_LED_PIN, HIGH); // 유저 LED 켜기
  delay(100); // 0.1초 유지
  digitalWrite(RED_LED_PIN, LOW); // 유저 LED 끄기
  delay(1000); // 1초 기다리기
}
