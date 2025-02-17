#ifndef DEBUG_H
#define DEBUG_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "Arduino.h"

//////////////  디버그 모드 설정 관련
#define DEBUG

#define DEBUG_OUTPUT_LEVEL_INFO
#define DEBUG_OUTPUT_LEVEL_DEBUG
#define DEBUG_OUTPUT_LEVEL_WARN
#define DEBUG_OUTPUT_LEVEL_ERROR

#define DEBUG_SERIAL        Serial
#define SERIAL_BAUD_RATE    115200



//////////////  Debug 함수
/* 
 * Debug용 통신을 초기화 하는 함수
 */
void initDebug() {
#ifdef DEBUG
  DEBUG_SERIAL.begin(SERIAL_BAUD_RATE);
#endif
}

#endif
