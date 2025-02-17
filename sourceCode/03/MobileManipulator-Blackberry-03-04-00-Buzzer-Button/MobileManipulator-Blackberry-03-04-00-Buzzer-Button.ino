#define pinBuzzer     5  // 부저 핀 번호

#define pinButton1    41 // 버턴1 핀 번호
#define pinButton2    40 // 버턴2 핀 번호

void  setup()
{
  pinMode( pinButton1, INPUT );  // 버튼1을 입력 모드로
  pinMode( pinButton2, INPUT );  // 버튼2를 입력 모드로
  
  pinMode( pinBuzzer, OUTPUT ); // 부저를 출력 모드로  
  noTone( pinBuzzer );  // 스피커 끄기(음소거)
}

void  loop()
{
  if( digitalRead(pinButton1) == 0 ) // 버튼1 눌림?
    tone( pinBuzzer, 261 ); // "도" 연주
  else if( digitalRead(pinButton2) == 0 ) // 버튼2 눌림?
    tone( pinBuzzer, 392 ); // "솔" 연주
  else
    noTone( pinBuzzer ); // 음소거(mute)
}
