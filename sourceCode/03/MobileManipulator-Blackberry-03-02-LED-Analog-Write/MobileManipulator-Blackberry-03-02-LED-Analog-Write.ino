//////////////  사용자 RED LED
#define RED_LED_PIN 4

void setup() {
  pinMode(RED_LED_PIN, OUTPUT); // LED 핀을 출력으로 설정
}

void loop() {
  for (int i = 0 ; i <= 255 ; i++) { // 서서히 밝아지기
    analogWrite(RED_LED_PIN, i);
    delay(10);
  }
  delay(500);
  
  for (int i = 255 ; i >= 0 ; i--) { // 서서히 어두워지기
    analogWrite(RED_LED_PIN, i);
    delay(10);
  }
  delay(500);
  
  for (int i = 0 ; i <= 255 ; i++) { // 더 빠르게 밝아지기
    analogWrite(RED_LED_PIN, i);
    delay(3);
  }
  delay(500);
  
  for (int i = 255 ; i >= 0 ; i--) { // 더 빠르게 어두워지기
    analogWrite(RED_LED_PIN, i);
    delay(3);
  }
  delay(500);
}
