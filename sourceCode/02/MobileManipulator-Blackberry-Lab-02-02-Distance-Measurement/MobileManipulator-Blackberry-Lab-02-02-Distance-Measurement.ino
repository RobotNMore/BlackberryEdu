////////////// [실습] 블랙 베리의 거리 감각은 진화 중.
////////////// 4개의 PSD 센서에서 측정한 아날로그 측정값의 5회 평균을 이용해,장애물
////////////// 까지의 실제 거리([cm])를 구한 후 0.5초 간격으로 Serial 모니터로 출력 

//////////////  거리 측정 관련 선언
// PIN_NUM는 전방 좌측, 전방 우측, 측방 좌측, 측방 우측
// PSD 센서의 연결핀 정보를 담고 있는 배열
const int PIN_NUM[] = { A0, A2, A1, A3 }; 

//////////////  메인 프로그램

void setup() {
  Serial.begin( 115200 ); // 시리얼 모니터 초기화 (baudrate)
}

void loop() {
  // 각 센서의 센서 값, 평균 값, 거리 값을 저장할 배열을 생성 및 초기화
  static int analogReadValue[5][4] = {0,}; // 배열을 0으로 채움
  static float analogReadValueAverage[4] = {0.0,};
  static float distanceValue[4] = {0.0,};

  for (int n=0; n<5; n++)  // 평균낼 횟수만큼 반복
  {
    for (int i=0; i<4; i++)  // 4개의 센서 값을 각각 읽어옴.
    {
      analogReadValue[n][i] = analogRead(PIN_NUM[i]); // PIN_NUM에 해당하는센서
                                                      // 값을 읽어옴.
    }
  }

  for (int i=0; i<4; i++)  //각 센서별 n회(여기선 5회) 측정값에 대해 평균을 냄.
  {
    analogReadValueAverage[i] = (analogReadValue[0][i]
                                 + analogReadValue[1][i]
                                 + analogReadValue[2][i]
                                 + analogReadValue[3][i]
                                 + analogReadValue[4][i])/5;
                                 
    // 센서별 인덱스와 아날로그 측정값 평균을 입력으로 psd2Distance 함수에서 처리하여,
    // 실제 거리([cm])를 distanceValue 배열에 저장
    distanceValue[i] = psd2Distance(i, analogReadValueAverage[i]);
  }

  // 아날로그 측정값 평균 출력( "[Analog Value(fl, fr, sl, sr)]: (a, b, c, d)" )
  Serial.print("[Analog Value(fl, fr, sl, sr)]: (");
  for (int i=0; i<4; i++)
  {
    Serial.print(analogReadValueAverage[i]);
    if (i<3){
      Serial.print(", ");
    }
    else{
      Serial.println(") ");
    }
  }
 
  // 아날로그 측정값 평균을 이용한 실제 거리값 출력
  // ( "[Distance(fl, fr, sl, sr)]: (a cm, b cm, c cm, d cm)" )
  Serial.print( "[Distance(fl, fr, sl, sr)] : (" );
  for (int i=0; i<4; i++)
  { 
    Serial.print( distanceValue[i], 1 );  //소수점 첫째자리까지 출력 
    if (i<3){ 
      Serial.print( " cm, " );
    }
    else {
      Serial.println( "cm )" );
    }
  }
  delay(500); 
}


// PSD 센서 인덱스와 아날로그 측정값 평균을 입력으로 장애물까지의
// 실제 거리([cm])를 출력하는 함수
// 각 센서의 실제 거리 값은 1cm 단위로 거리를 변화시켜가며 측정한 아날로그 신호를 이용해,
// 아날로그 신호에 대한 거리값 그래프를 그린후 거듭제곱 형태의 추세 함수를 추정하여 구함.
float psd2Distance(int x, float d) 
{
  float distance;                     // 아날로그 관측값 (5회) 평균을 이용해
                                      // 산출한 실제 거리([cm])
  
  if (x==0){
    distance = 13768*pow(d, -1.183);   // 전방 좌측 거리 추세 함수
  }
  else if (x==1){
    distance = 15911*pow(d, -1.2);     // 전방 우측 거리 추세 함수
  }
  else if (x==2){
    distance = 37406*pow(d, -1.333);    // 측방 좌측 거리 추세 함수
  }
  else if (x==3){
    distance = 44932*pow(d, -1.361);     // 측방 우측 거리 추세 함수
  }
  else {
    distance = 0.0;
  }
  return distance;
}
