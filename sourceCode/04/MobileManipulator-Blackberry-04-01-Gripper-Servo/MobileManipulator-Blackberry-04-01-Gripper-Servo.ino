//////////////  픽시 카메라 모듈에 연결된 그리퍼를 오므리고 벌리기
#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;

#define GRIP_ANGLE_CLOSE          650  // 그리퍼 반 정도 닫히는 값(1000이 최대)
#define GRIP_ANGLE_OPEN           100  // 그리퍼 열리는 값


//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  
  pixy.init(); // 카메라 모듈 초기화
  pixy.setServos(0, GRIP_ANGLE_OPEN); // 그리퍼 벌리기(100) 위치로 서보 모터 이동
}

void loop()
{
  pixy.setServos(0, GRIP_ANGLE_CLOSE); // 그리퍼 오므리기(650) 위치로 서보 모터 이동
  delay(800); // 서보 모터가 설정된 위치로 이동하기 까지 시간대기

  delay(1000);  // 1초 대기
  
  pixy.setServos(0, GRIP_ANGLE_OPEN); // 그리퍼 벌리기(100) 위치로 서보 모터 이동
  delay(800); // 서보 모터가 설정된 위치로 이동하기 까지 시간대기

  delay(3000);  // 3초 대기 후 다시 반복
}
