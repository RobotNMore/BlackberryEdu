void setup() {  
    pinMode(4, OUTPUT); // LED
    pinMode(41, INPUT); // SW (왼쪽 버튼)
}

void loop() {
    if( digitalRead(41) == 0 )  // 버튼이 눌렸으면
        digitalWrite(4, 1); // LED ON
    else
        digitalWrite(4, 0); // LED OFF
    
    delay(100);
}
