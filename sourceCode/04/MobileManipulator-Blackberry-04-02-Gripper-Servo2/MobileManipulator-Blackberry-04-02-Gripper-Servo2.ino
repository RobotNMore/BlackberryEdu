//////////////  픽시 카메라 모듈에 연결된 그리퍼를 서서히 오므렸다 벌리기
#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;

#define GRIP_ANGLE_CLOSE          650  // 그리퍼 반 정도 닫히는 값(1000이 최대)
#define GRIP_ANGLE_OPEN           100  // 그리퍼 열리는 값

int pos = 0; // 서보모터 제어 값 변수

//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  
  pixy.init(); // 카메라 모듈 초기화
  pixy.setServos(0, GRIP_ANGLE_OPEN); // 그리퍼 열기
}

void loop()
{
  // 서보 모터 제어 값을 100에서 650 까지 2씩 증가시키면서 그리퍼를 서서히 오므림
  for (pos = GRIP_ANGLE_OPEN; pos <= GRIP_ANGLE_CLOSE; pos += 2) {
    pixy.setServos(0, pos); // 서보 모터를 해당 위치(각도)로 이동
    delay(25); // 서보 모터 이동 대기 시간 (시간이 길수록 천천히 이동함)
  }
  
  // 서보 모터 제어 값을 650에서 100 까지 2씩 감소시키면서 그리퍼를 서서히 벌림
  for (pos = GRIP_ANGLE_CLOSE; pos >= GRIP_ANGLE_OPEN; pos -= 2) {
    pixy.setServos(0, pos);
    delay(25);
  }

  delay(5000);  // 5초 대기 후 다시 반복
}
