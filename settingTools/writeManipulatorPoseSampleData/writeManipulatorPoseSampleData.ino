#include <EEPROM.h>

#define DATA_SIZE     560
uint8_t manipulatorPoseData[DATA_SIZE] = {1, 1, 96, 3, 166, 8, 105, 6, 183, 2, 73, 110, 105, 116, 105, 97, 108, 32, 38, 32, 77, 105, 115, 115, 105, 111, 110, 32, 73, 110, 115, 116, 114, 117, 99, 116, 105, 111, 110, 0,
                                          1, 2, 251, 7, 250, 9, 147, 3, 172, 4, 115, 101, 101, 32, 98, 108, 111, 99, 107, 32, 115, 116, 111, 114, 97, 103, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 3, 213, 7, 229, 9, 16, 3, 163, 6, 114, 101, 97, 100, 121, 32, 116, 111, 32, 103, 114, 105, 112, 32, 117, 112, 112, 101, 114, 32, 98, 108, 111, 99, 107, 0, 0, 0, 0, 0,
                                          1, 4, 226, 7, 250, 7, 39, 4, 112, 7, 103, 114, 105, 112, 32, 117, 112, 112, 101, 114, 32, 98, 108, 111, 99, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 5, 220, 7, 139, 10, 186, 1, 211, 6, 114, 101, 97, 100, 121, 32, 116, 111, 32, 103, 114, 105, 112, 32, 108, 111, 119, 119, 101, 114, 32, 98, 108, 111, 99, 107, 0, 0, 0, 0,
                                          1, 6, 226, 7, 188, 7, 105, 2, 103, 9, 103, 114, 105, 112, 32, 108, 111, 119, 101, 114, 32, 98, 108, 111, 99, 107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 7, 124, 2, 131, 5, 212, 6, 225, 7, 112, 117, 116, 49, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 8, 19, 3, 89, 5, 214, 6, 225, 7, 112, 117, 116, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 9, 168, 3, 210, 5, 27, 6, 225, 7, 112, 117, 116, 51, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 10, 90, 4, 34, 6, 125, 5, 225, 7, 112, 117, 116, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 11, 32, 2, 246, 5, 140, 5, 225, 7, 112, 117, 116, 53, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 12, 185, 2, 253, 6, 111, 4, 225, 7, 112, 117, 116, 54, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 13, 142, 3, 92, 7, 37, 4, 225, 7, 112, 117, 116, 55, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                          1, 14, 91, 4, 154, 7, 26, 4, 225, 7, 112, 117, 116, 56, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

                               
int address = 0;

void setup() {
  Serial.begin(115200);
  Serial.print("start writing ");
  Serial.print(DATA_SIZE);
  Serial.println("bytes");
}

void loop() {
  EEPROM.update(address, manipulatorPoseData[address]);
  
  Serial.print('.');
  if ((address - 39)%40 == 0)
    Serial.println();
  
  address = address + 1;
  
  if (address == DATA_SIZE) {
    Serial.println("complete writing");
    while(1) { delay(1000); }
  }
  
  delay(50);
}
