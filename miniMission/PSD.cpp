#include "Arduino.h"
#include "Debug.h"
#include "Pins.h"
#include "PSD.h"


//////////////  PSD 함수 정의
void InitPSD() {
  // 아날로그 핀은 핀모드 설정 필요 없음
}

void GetValueFromFrontPSDSensors(int16_t* flPSDValuePtr, int16_t* frPSDValuePtr) {
  *flPSDValuePtr = analogRead(PIN_FRONT_LEFT_PSD);
  *frPSDValuePtr = analogRead(PIN_FRONT_RIGHT_PSD);
    
#if defined(DEBUG) & 0
  DEBUG_SERIAL.print("PSD/getValueFromFrontPSDSensors : flPSDValue : ");
  DEBUG_SERIAL.println(*flPSDValuePtr);
  DEBUG_SERIAL.print("PSD/getValueFromFrontPSDSensors : flPSDValue : ");
  DEBUG_SERIAL.println(*frPSDValuePtr);
#endif
}

void GetValueFromFrontLeftPSDSensor(int16_t* flPSDValuePtr) {
  *flPSDValuePtr = analogRead(PIN_FRONT_LEFT_PSD);
    
#if defined(DEBUG) & 0
  DEBUG_SERIAL.print("PSD/getValueFromFrontLeftPSDSensors : flPSDValuePtr : ");
  DEBUG_SERIAL.println(*flPSDValuePtr);
#endif
}

void GetValueFromFrontRightPSDSensor(int16_t* frPSDValuePtr) {
  *frPSDValuePtr = analogRead(PIN_FRONT_RIGHT_PSD);
    
#if defined(DEBUG) & 0
  DEBUG_SERIAL.print("PSD/getValueFromFrontRightPSDSensors : frPSDValuePtr : ");
  DEBUG_SERIAL.println(*frPSDValuePtr);
#endif
}

void GetValueFromSideLeftPSDSensor(int16_t* slPSDValuePtr) {
  *slPSDValuePtr = analogRead(PIN_SIDE_LEFT_PSD);
    
#if defined(DEBUG) & 0
  DEBUG_SERIAL.print("PSD/getValueFromSideLeftPSDSensors : slPSDValue : ");
  DEBUG_SERIAL.println(*slPSDValuePtr);
#endif
}

void GetValueFromSideRightPSDSensor(int16_t* srPSDValuePtr) {
  *srPSDValuePtr = analogRead(PIN_SIDE_RIGHT_PSD);
    
#if defined(DEBUG) & 0
  DEBUG_SERIAL.print("PSD/getValueFromSideRightPSDSensors : srPSDValue : ");
  DEBUG_SERIAL.println(*srPSDValuePtr);
#endif
}
