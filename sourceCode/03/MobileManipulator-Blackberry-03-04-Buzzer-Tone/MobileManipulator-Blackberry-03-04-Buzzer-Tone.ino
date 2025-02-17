//////////////  부저
#define BUZZER_PIN            5

#define OCTAVE_4_DO           262
#define OCTAVE_4_RE           294
#define OCTAVE_4_MI           330
#define OCTAVE_4_SOL          392
#define OCTAVE_4_LA           440
#define OCTAVE_4_TI           494
#define OCTAVE_5_DO           523

#define WHOLE_NOTE            1300
#define HALF_NOTE             WHOLE_NOTE/2
#define QUATER_NOTE           WHOLE_NOTE/4
#define DOTTED_QUATER_NOTE    WHOLE_NOTE*3/8
#define EIGHTH_NOTE           WHOLE_NOTE/8
#define SIXTEENTH_NOTE        WHOLE_NOTE/16
#define SHORT_DELAY           10

int melody[] = {OCTAVE_4_MI, OCTAVE_4_RE, OCTAVE_4_DO, OCTAVE_4_RE,
                OCTAVE_4_MI, OCTAVE_4_MI, OCTAVE_4_MI,
                OCTAVE_4_RE, OCTAVE_4_RE, OCTAVE_4_RE,
                OCTAVE_4_MI, OCTAVE_4_SOL, OCTAVE_4_SOL,
                OCTAVE_4_MI, OCTAVE_4_RE, OCTAVE_4_DO, OCTAVE_4_RE,
                OCTAVE_4_MI, OCTAVE_4_MI, OCTAVE_4_MI,
                OCTAVE_4_RE, OCTAVE_4_RE, OCTAVE_4_MI, OCTAVE_4_RE,
                OCTAVE_4_DO};
int note[] = {DOTTED_QUATER_NOTE, EIGHTH_NOTE, QUATER_NOTE, QUATER_NOTE,
              QUATER_NOTE, QUATER_NOTE, HALF_NOTE,
              QUATER_NOTE, QUATER_NOTE, HALF_NOTE,
              QUATER_NOTE, QUATER_NOTE, HALF_NOTE,
              DOTTED_QUATER_NOTE, EIGHTH_NOTE, QUATER_NOTE, QUATER_NOTE,
              QUATER_NOTE, QUATER_NOTE, HALF_NOTE,
              QUATER_NOTE, QUATER_NOTE, QUATER_NOTE, QUATER_NOTE,
              WHOLE_NOTE};

int prevNote = 0;

void setup() {
  if (sizeof(melody) != sizeof(note))
    while(1) {};  // 두 배열의 사이즈가 다르면 연주할 수 없음
    
  for (int i = 0 ; i < sizeof(melody)/sizeof(int) ; i++) {
    if (prevNote == melody[i]) { // 현재 출력하려는 음이 이전 음과 같을 때
      noTone(BUZZER_PIN); // 잠깐 끊어서 구분함
      delay(SHORT_DELAY);
    }
    tone(BUZZER_PIN, melody[i]);
    delay(note[i]);
    prevNote = melody[i];
  }
  noTone(BUZZER_PIN);
}

void loop() {
}
