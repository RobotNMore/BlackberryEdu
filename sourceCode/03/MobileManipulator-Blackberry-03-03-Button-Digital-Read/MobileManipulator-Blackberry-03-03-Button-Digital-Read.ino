//////////////  사용자 RED LED
#define RED_LED_PIN 4

//////////////  사용자 Swtich
#define SW1_PIN       41
#define SW2_PIN       40

void setup() {
  Serial.begin(115200); // 시리얼모니터 통신 설정
  
  pinMode(RED_LED_PIN, OUTPUT); // LED 핀을 출력으로 설정
  
  pinMode(SW1_PIN, INPUT); // SW1 핀을 입력으로 설정
  pinMode(SW2_PIN, INPUT); // SW2 핀을 입력으로 설정
}

void loop() {
    // 스위치가 눌리면 1, 아니면 0이 출력되도록 digitalRead에 !연산을 함
    Serial.print("SW1 : ");
    Serial.print(!digitalRead(SW1_PIN));
    Serial.print(", SW2 : ");
    Serial.println(!digitalRead(SW2_PIN));

    // 두 스위치의 상태가 서로 다를 때만 LED를 켜도록 함
    digitalWrite(RED_LED_PIN, digitalRead(SW1_PIN) != digitalRead(SW2_PIN));
    
    delay(50);
}
