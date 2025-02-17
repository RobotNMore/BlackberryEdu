////////////// [실습] 조명과 음악의 랑데부, 빛 연주가 블랙 베리

//////////////  스위치
#define LEFT_SWITCH_PIN       41
#define RIGHT_SWITCH_PIN      40

//////////////  부저
#define BUZZER_PIN            5

//////////////  RGB LED
#include <Adafruit_NeoPixel.h>
#define RGB_LED_PIN           3
#define NUM_PIXELS            1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS, RGB_LED_PIN,
                                             NEO_GRBW + NEO_KHZ800 );

#define DO_3                  131
#define RE_3                  147
#define MI_3                  165
#define FA_3                  175
#define SOL_3                 196
#define LA_3                  220
#define TI_3                  247
#define DO_4                  262
#define RE_4                  294
#define MI_4                  330
#define FA_4                  349


#define n4                    1300
#define n2                    n4/2
#define n1                    n4/4
#define nd2                   n4/8              
#define SHORT_DELAY           10


int melody[] = {SOL_3, DO_4, DO_4, RE_4, DO_4, TI_3, LA_3, LA_3,
                LA_3, RE_4, RE_4, MI_4, RE_4, DO_4, TI_3, SOL_3,
                SOL_3, MI_4, MI_4, FA_4, MI_4, RE_4, DO_4, LA_3,
                SOL_3, SOL_3, LA_3, RE_4, TI_3, DO_4 };
int note[] = {n1, n1, nd2, nd2, nd2, nd2, n1, n1,
              n1, n1, nd2, nd2, nd2, nd2, n1, n1,
              n1, n1, nd2, nd2, nd2, nd2, n1, n1,
              nd2, nd2, n1, n1, n1, n1};
int prevNote = 0;


void setup() {
  if (sizeof(melody) != sizeof(note))
    while(1) {};  // 두 배열의 사이즈가 다르면 연주할 수 없음
    
  pixels.begin();
  pinMode(LEFT_SWITCH_PIN, INPUT);
  pinMode(RIGHT_SWITCH_PIN, INPUT);
}


void loop() {
  if (!digitalRead(LEFT_SWITCH_PIN)) {  // 왼쪽 버튼을 누르면 연주 시작
    for (int i = 0 ; i < sizeof(melody)/sizeof(int) ; i++) {
      if (prevNote == melody[i]) { // 현재 출력하려는 음이 이전 음과 같을 때
        noTone(BUZZER_PIN); // 잠깐 끊어서 구분함
        pixels.setBrightness(0);  // LED를 끔
        pixels.show();
        delay(SHORT_DELAY);
      }

      uint32_t soundStartTime = millis();
      bool isWaiting = false;
      bool isStopButtonPressed = false;

      while(millis() - soundStartTime < note[i]) {
        // 오른쪽 버튼을 눌러 연주 종료
        if (!digitalRead(RIGHT_SWITCH_PIN)) {
          isStopButtonPressed = true;
          break;  // while 문을 종료
        }
          
        if (!isWaiting) {
          // 음악을 연주
          tone(BUZZER_PIN, melody[i]);
          // LED 색상을 랜덤하게 표현
          pixels.setPixelColor(0, pixels.Color(random(1, 256),
                                               random(1, 256),
                                               random(1, 256)));
          pixels.setBrightness(100);
          pixels.show();
          prevNote = melody[i];

          isWaiting = true;
        } else {
          delay(1);
        }
      }
    }
    noTone(BUZZER_PIN);
    pixels.setBrightness(0);
    pixels.show();
  }
}
