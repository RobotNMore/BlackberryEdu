//////////////  전방 PSD센서

#define PIN_FRONT_LEFT_PSD    A0 // 매니퓰레이터 전방 좌측 PSD센서 연결 핀
#define PIN_FRONT_RIGHT_PSD   A2 // 매니퓰레이터 전방 우측 PSD센서 연결 핀
 
#define LEFT_PSD_INDEX    0  // readings변수에서 좌측 PSD값이 저장된 인덱스
#define RIGHT_PSD_INDEX   1  // readings변수에서 우측 PSD값이 저장된 인덱스

#define PSD_CNT           2  // PSD가 두 개 있음 (좌/우)

//////////////  이동 평균 필터

#define NUM_READINGS      100 // 좌/우 각 PSD로 읽어서 저장할 데이터 수

float readings[PSD_CNT][NUM_READINGS]; // 읽어들인 PSD값
int16_t readIndex = 0;      // PSD값이 저장될 인덱스
float total[PSD_CNT];       // readings에 있는 값들의 합
float average[PSD_CNT];     // readings에 있는 값들의 평균

//////////////  시리얼모니터 출력 관련

#define PRINT_RAW_SENSOR_VALUE

void setup() {
  Serial.begin(115200);

  // readings, total, average 초기화
  for (int16_t thisPSD = 0; thisPSD < PSD_CNT; thisPSD++) // 좌/우 PSD에 대해
  {
    for (int16_t thisReading = 0; thisReading < NUM_READINGS; thisReading++) // 각 100번씩
    {
       readings[thisPSD][thisReading] = 0; // 값을 0으로 초기화
    }
    total[thisPSD] = 0;
    average[thisPSD] = 0;
  }
}

void loop() {
  // 가장 예전에 읽은 PSD값을 total에서 뺌
  total[LEFT_PSD_INDEX] = total[LEFT_PSD_INDEX] - readings[LEFT_PSD_INDEX][readIndex];
  total[RIGHT_PSD_INDEX] = total[RIGHT_PSD_INDEX] - readings[RIGHT_PSD_INDEX][readIndex];

  // 센서 값을 읽어서 readings의 가장 나중에 읽은 값을 대체
  //readings[LEFT_PSD_INDEX][readIndex] = 1/((analogRead( PIN_FRONT_LEFT_PSD )*2.059e-4) - 1.473e-3);
  //readings[RIGHT_PSD_INDEX][readIndex] = 1/((analogRead( PIN_FRONT_RIGHT_PSD )*2.773e-4) - 3.84e-2);

  readings[LEFT_PSD_INDEX][readIndex] = 13768 * pow(analogRead( PIN_FRONT_LEFT_PSD ), -1.183);
  readings[RIGHT_PSD_INDEX][readIndex] = 15911 * pow(analogRead( PIN_FRONT_RIGHT_PSD ), -1.2);

#ifdef PRINT_RAW_SENSOR_VALUE
  Serial.print("left_PSD:");
  Serial.print(readings[LEFT_PSD_INDEX][readIndex]);
  Serial.print(",right_PSD:");
  Serial.print(readings[RIGHT_PSD_INDEX][readIndex]);
#endif
  
  // 가장 최근에 읽은 PSD값을 total에 더함
  total[LEFT_PSD_INDEX] = total[LEFT_PSD_INDEX] + readings[LEFT_PSD_INDEX][readIndex];
  total[RIGHT_PSD_INDEX] = total[RIGHT_PSD_INDEX] + readings[RIGHT_PSD_INDEX][readIndex];
  
  readIndex = readIndex + 1;
  if (readIndex >= NUM_READINGS) { // readIndex가 배열 크기보다 커지면
    readIndex = 0; // 다시 배열의 처음을 가리키게 함
  }

  // 평균을 계산
  average[LEFT_PSD_INDEX] = total[LEFT_PSD_INDEX] / NUM_READINGS;
  average[RIGHT_PSD_INDEX] = total[RIGHT_PSD_INDEX] / NUM_READINGS;

#ifdef PRINT_RAW_SENSOR_VALUE
  Serial.print(",left_average:");
#else
  Serial.print("left_average:");
#endif
  Serial.print(average[LEFT_PSD_INDEX]);
  Serial.print(",right_average:");
  Serial.println(average[RIGHT_PSD_INDEX]);
}
