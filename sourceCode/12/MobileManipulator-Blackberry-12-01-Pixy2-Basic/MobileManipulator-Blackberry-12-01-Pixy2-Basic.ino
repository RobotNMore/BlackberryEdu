//////////////  Pixy2 카메라
#include <Pixy2SPI_SS.h> // SPI 통신하는 Pixy2 라이브러리를 포함시킴

Pixy2SPI_SS pixy;        // SPI 통신하는 Pixy2 객체 생성

//////////////  메인 프로그램

void setup()
{
  Serial.begin(115200);
  
  pixy.init(); // pixy 객체 초기화
}

void loop()
{
  pixy.ccc.getBlocks(); // ccc는 "color connected components"로
                        // getBlocks()를 호출하여 인식된 사물에 대한 정보를
                        // pixy.ccc.blocks에 업데이트 함
                        
  if( pixy.ccc.numBlocks ) // numBlocks는 인식된 블록의 개수
  {                        // 만약 인식된 블록이 있을 때만 아래 내용을 실행
    
    Serial.print("Detected "); // 블록이 감지되었음을 출력
    Serial.println(pixy.ccc.numBlocks); // 객체(블록)가 몇 개 감지되었는지 출력
    
    for(int i=0; i<pixy.ccc.numBlocks; i++ ) // 인식 된 블록 개수만큼 반복
    {
        Serial.print("  block "); 
        Serial.print(i); // 몇 번째 블록인지 출력. 블록이 정렬되는(번호) 순서는
                         // 인식된 블록 영역(width*height, 크기)의 내림차순
        Serial.print(": ");
        pixy.ccc.blocks[i].print(); // 인식된 블록의 정보를 시리얼 모니터에 출력
    }
  }
}
