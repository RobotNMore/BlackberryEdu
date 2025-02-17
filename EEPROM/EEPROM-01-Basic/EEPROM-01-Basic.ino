////////////////////  EEPROM 데이터 쓰기/읽기

#include <EEPROM.h>

#define EEPROM_TEST_FLOAT     240 // 테스트용 데이터 저장 시작 위치(offset)
#define EEPROM_TEST_INT1      (EEPROM_TEST_FLOAT + 4) // 실수형= 4바이트
#define EEPROM_TEST_INT2      (EEPROM_TEST_INT1 + 2)  // 정수형= 2바이트

//////////////// 테스트 데이터 (실수4 + 정수2 + 정수2 = 8바이트)

float FloatData = 3.14; // 실수형 데이터는 4바이트 차지함
int   Integer1Data = -12345; // 정수형 데이터는 2바이트 차지함
unsigned int   Integer2Data = 65535; // 0 ~ 65535 (2바이트)

void  WriteData() // EEPROM에 데이터 쓰기
{
  // 데이터 유형에 따라 자동으로 저장할 때 --> put 함수 사용
  
  EEPROM.put( EEPROM_TEST_FLOAT, FloatData );
  EEPROM.put( EEPROM_TEST_INT1, Integer1Data );

  // 데이터를 1-BYTE 단위로 분리해서 저장할 때 --> write 함수 사용
  
  byte  byte0 = Integer2Data & 0x00FF;  // 하위 바이트
  byte  byte1 = (Integer2Data & 0xFF00) >> 8; // 상위 바이트

  EEPROM.write( EEPROM_TEST_INT2 + 0, byte0 );
  EEPROM.write( EEPROM_TEST_INT2 + 1, byte1 );
}

void  ReadData()  // EEPROM에 저장된 데이터 읽기
{
  // 데이터 유형에 따라 자동으로 값을 읽을 때 --> get 함수 사용
  
  EEPROM.get( EEPROM_TEST_FLOAT, FloatData );
  EEPROM.get( EEPROM_TEST_INT1, Integer1Data );

  // 데이터를 BYTE 단위로 분리해서 값을 읽을 때 --> read 함수 사용
 
  byte  byte0 = EEPROM.read( EEPROM_TEST_INT2 + 0 );
  byte  byte1 = EEPROM.read( EEPROM_TEST_INT2 + 1 );

  Integer2Data = ((unsigned int)byte1 << 8) + byte0;
}

////////////////////  메인 프로그램 (setup & loop)

void setup() 
{
  Serial.begin( 9600 ); // 시리얼 통신 속도 설정

  Serial.print( "Writing values to EEPROM : " );
  WriteData(); // EEPROM에 데이터 쓰기
  Serial.println( "done!" );

  // EEPROM에 쓴 값들을 읽기 전에 모두 0으로 초기화
  FloatData = 0.0;
  Integer1Data = 0;
  Integer2Data = 0;
  Serial.println( "All values are initialized to 0." );
  
  Serial.print( "Reading values from EEPROM : " );
  ReadData();  // EEPROM에 저장된 데이터 읽기
  Serial.println( "done!" );
  
  Serial.print( "FloatData= " );  // 읽은 데이터 값들 출력
  Serial.print( FloatData );
  Serial.print( ", Integer1Data= " );
  Serial.print( Integer1Data );
  Serial.print( ", Integer2Data= " );
  Serial.println( Integer2Data );
  Serial.println();
}

void loop()
{
}
