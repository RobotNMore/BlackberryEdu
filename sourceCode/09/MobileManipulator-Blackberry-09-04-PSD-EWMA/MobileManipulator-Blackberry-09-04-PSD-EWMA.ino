//////////////  전방 PSD센서

#define PIN_FRONT_LEFT_PSD    A0 // 매니퓰레이터 전방 좌측 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 매니퓰레이터 전방 우측 PSD센서 연결 핀

#define LEFT_PSD_INDEX    0  // readings변수에서 좌측 PSD값이 저장된 인덱스
#define RIGHT_PSD_INDEX   1  // readings변수에서 우측 PSD값이 저장된 인덱스

#define PSD_CNT           2  // PSD가 두 개 있음 (좌/우)

//////////////  지수가중이동평균필터

const float prevValueWeight = 0.95; // 값이 크면 필터링 효과 커짐
float distance[PSD_CNT];            // 센서값으로 구해진 거리
float result[PSD_CNT];              // 결과 값

//////////////  시리얼모니터 출력 관련

#define PRINT_RAW_SENSOR_VALUE

void setup() {
  Serial.begin(115200);

  // distance, result 초기화
  for (int16_t thisPSD = 0; thisPSD < PSD_CNT; thisPSD++)
  {
    distance[thisPSD] = 0;
    result[thisPSD] = 0;
  }
}

void loop() {
  // 현재 거리 값 계산
  //distance[LEFT_PSD_INDEX] = 1/((analogRead( PIN_FRONT_LEFT_PSD )*2.059e-4) - 1.473e-3);
  //distance[RIGHT_PSD_INDEX] = 1/((analogRead( PIN_FRONT_RIGHT_PSD )*2.773e-4) - 3.84e-2);

  distance[LEFT_PSD_INDEX] = 13768 * pow(analogRead( PIN_FRONT_LEFT_PSD ), -1.183);
  distance[RIGHT_PSD_INDEX] = 15911 * pow(analogRead( PIN_FRONT_RIGHT_PSD ), -1.2);

  // 이전 결과 값 비율을 줄이고 현재 거리 값을 결과에 반영하여 결과에 저장
  result[LEFT_PSD_INDEX] = prevValueWeight*result[LEFT_PSD_INDEX]
                            + (1 - prevValueWeight)*distance[LEFT_PSD_INDEX];
  result[RIGHT_PSD_INDEX] = prevValueWeight*result[RIGHT_PSD_INDEX]
                            + (1 - prevValueWeight)*distance[RIGHT_PSD_INDEX];

#ifdef PRINT_RAW_SENSOR_VALUE
  Serial.print("left_PSD:");
  Serial.print(distance[LEFT_PSD_INDEX]-20); // 그래프가 겹치면 잘 안보여서
  Serial.print(",right_PSD:");              // 그래프를 내리기 위해 임의의 값을 뺌
  Serial.print(distance[RIGHT_PSD_INDEX]-20);
  Serial.print(",left_result:");
#else
  Serial.print("left_result:");
#endif
  Serial.print(result[LEFT_PSD_INDEX]);
  Serial.print(",right_result:");
  Serial.println(result[RIGHT_PSD_INDEX]);
}
