//////////////  RGB LED
#include <Adafruit_NeoPixel.h>
#define RGB_LED_PIN 3
#define NUM_PIXELS  1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXELS,
                                             RGB_LED_PIN,
                                             NEO_GRBW + NEO_KHZ800);

void setup() {
  pixels.begin(); // RGB LED 설정
  pixels.setBrightness(15); // 밝기 설정
  pixels.show(); // 변경사항을 보이기
}

void loop() {
  pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // 빨강
  pixels.show();
  delay(1000); // 1초 유지
  
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // 초록
  pixels.show();
  delay(1000); // 1초 유지
  
  pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // 파랑
  pixels.show();
  delay(1000); // 1초 유지

  
  pixels.setPixelColor(0, pixels.Color(255, 255, 0)); // 노랑
  pixels.show();
  delay(1000); // 1초 유지
  
  pixels.setPixelColor(0, pixels.Color(0, 255, 255)); // 시안
  pixels.show();
  delay(1000); // 1초 유지
  
  pixels.setPixelColor(0, pixels.Color(255, 0, 255)); // 마젠타
  pixels.show();
  delay(1000); // 1초 유지
  
  
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // 흰색
  pixels.show();
  delay(1000); // 1초 유지
}
