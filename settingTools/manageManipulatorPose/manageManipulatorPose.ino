/*
 * 보드에 최소한 한 번은
 * EEPROM 샘플 코드의 eeprom_clear를 실행해 주어야 함
 * 쓰레기 값이 들어있는 것 같음
 */

#include "Debug.h"
#include "Motor.h"
#include "Manipulator.h"
#include "RGBLED.h"

enum CommandStatus {
  COMMAND,
  SAVE,     // id, description
  READ,     // id
  REMOVE,   // id,
  RUN       // id, operatingTime, motor1Angle(optional)
};

CommandStatus currentCommandStatus = COMMAND;

#define SAVE_COMMAND_PARAMETER_CNT            2
#define READ_COMMAND_PARAMETER_CNT            1
#define REMOVE_COMMAND_PARAMETER_CNT          1
#define RUN_COMMAND_PARAMETER_MIN_CNT         2
#define RUN_COMMAND_PARAMETER_MAX_CNT         3

#define RECEIVE_BUFFER_SIZE   40
char receiveBuffer[RECEIVE_BUFFER_SIZE] = {0,};
uint8_t receiveBufferIdx = 0;

const int32_t LONG_MAX = 2147483647L;

//////////////  메인 프로그램


void setup()
{
  initDebug(); // 디버그를 사용하는 다른 것들보다 먼저 해주어야 함
//  initMotorCommunication();  // initManipulator, initMobileBase보다 먼저 해 주어야 함
//  while(!initManipulator()) {}
//  initRGBLED();

  printSelectCommandGuide();

  delay(3000);
}

void loop()
{
  if (DEBUG_SERIAL.available()) {
    while(1) {
      if (DEBUG_SERIAL.available()) {
        char receivedData = DEBUG_SERIAL.read();
        if (receivedData == '\n') {
          flushDebugSerialData();
          break;
        }
        if (receiveBufferIdx == RECEIVE_BUFFER_SIZE) {
          DEBUG_SERIAL.println(F("receive buffer over flow"));
          DEBUG_SERIAL.println();
          delay(200); // 더 들어올 때 까지 기다려봄
          resetAllReceivedData();
          return;
        }
        receiveBuffer[receiveBufferIdx++] = receivedData;
      }
    }
    String receivedString = String(receiveBuffer);
    
    if (currentCommandStatus == COMMAND) {
      if (receivedString.equalsIgnoreCase("on")) {
        turnOnManipulatorTorque();
        DEBUG_SERIAL.println(F("torque on"));
        DEBUG_SERIAL.println();
      } else if (receivedString.equalsIgnoreCase("off")) {
        turnOffManipulatorTorque();
        DEBUG_SERIAL.println(F("torque off"));
        DEBUG_SERIAL.println();
      } else if (receivedString.equalsIgnoreCase("list")) {
        DEBUG_SERIAL.println(F("show pose list"));
        DEBUG_SERIAL.println();
        PrintManipulatorPoseListFromEEPROM();
      } else if (receivedString.equalsIgnoreCase("save")) {
        currentCommandStatus = SAVE;
        DEBUG_SERIAL.println(F("save pose"));
        DEBUG_SERIAL.println();
        printTypeSaveCommandParameterGuide();
      } else if (receivedString.equalsIgnoreCase("read")) {
        currentCommandStatus = READ;
        DEBUG_SERIAL.println(F("read pose"));
        DEBUG_SERIAL.println();
        printTypeReadCommandParameterGuide();
      } else if (receivedString.equalsIgnoreCase("remove")) {
        currentCommandStatus = REMOVE;
        DEBUG_SERIAL.println(F("remove pose"));
        DEBUG_SERIAL.println();
        printTypeRemoveCommandParameterGuide();
      } else if (receivedString.equalsIgnoreCase("run")) {
        currentCommandStatus = RUN;
        DEBUG_SERIAL.println(F("run pose"));
        DEBUG_SERIAL.println();
        printTypeRunCommandParameterGuide();
      } else if (receivedString.equalsIgnoreCase("help")) {
        printSelectCommandGuide();
      } else {
        DEBUG_SERIAL.println(F("wrong command"));
        DEBUG_SERIAL.println();
        printSelectCommandGuide();
      }
      resetAllReceivedData();
      
    } else if (currentCommandStatus == SAVE) {
      if (receivedString.equalsIgnoreCase("help")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("save command : help"));
        DEBUG_SERIAL.println();
        printTypeSaveCommandParameterGuide();
        return;
      } else if (receivedString.equalsIgnoreCase("quit")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("save command : quit"));
        DEBUG_SERIAL.println();
        currentCommandStatus = COMMAND;
        printSelectCommandGuide();
        return;
      }
      
      // 파라미터 개수 체크
      uint8_t paramCnt = 1;
      for (int i = 0 ; i < receiveBufferIdx ; i++) {
        if (receiveBuffer[i] == ',') paramCnt++;
      }
      if (paramCnt != SAVE_COMMAND_PARAMETER_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("save command : wrong parameters count, it needs two parameters"));
        DEBUG_SERIAL.println();
        printTypeSaveCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      uint8_t substringFromIdx = 0;
      uint8_t substringToIdx = receivedString.indexOf(',', substringFromIdx);
      uint8_t id = receivedString.substring(substringFromIdx, substringToIdx).toInt();
      if (id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("save command : wrong id"));
        DEBUG_SERIAL.println();
        printTypeSaveCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      substringFromIdx = substringToIdx+1;
      substringToIdx = receivedString.length();
      String description = receivedString.substring(substringFromIdx, substringToIdx);
      if (description.length() > 30) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("save command : description size overflow"));
        DEBUG_SERIAL.println();
        printTypeSaveCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      WriteManipulatorPresentPoseToEEPROM(id, description);
      
      resetAllReceivedData();
      DEBUG_SERIAL.println(F("save command : pose is saved"));
      DEBUG_SERIAL.println();
      turnOnLEDGreen500ms();
      
    } else if (currentCommandStatus == READ) {
      if (receivedString.equalsIgnoreCase("help")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("read command : help"));
        DEBUG_SERIAL.println();
        printTypeReadCommandParameterGuide();
        return;
      } else if (receivedString.equalsIgnoreCase("quit")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("read command : quit"));
        DEBUG_SERIAL.println();
        currentCommandStatus = COMMAND;
        printSelectCommandGuide();
        return;
      }
      
      // 파라미터 개수 체크
      uint8_t paramCnt = 1;
      for (int i = 0 ; i < receiveBufferIdx ; i++) {
        if (receiveBuffer[i] == ',') paramCnt++;
      }
      if (paramCnt != READ_COMMAND_PARAMETER_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("read command : wrong parameters count, it needs one parameter"));
        DEBUG_SERIAL.println();
        printTypeReadCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      uint8_t substringFromIdx = 0;
      uint8_t substringToIdx = receivedString.length();
      uint8_t id = receivedString.substring(substringFromIdx, substringToIdx).toInt();
      if (id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("read command : wrong id"));
        DEBUG_SERIAL.println();
        printTypeReadCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM(id);
      
      resetAllReceivedData();
      
      if (manipulatorPose.isTherePoseData) {
        DEBUG_SERIAL.println(F("read command : success"));
        DEBUG_SERIAL.print(F("   id : "));
        DEBUG_SERIAL.println(manipulatorPose.id);
        DEBUG_SERIAL.print(F("   manipulatorMotor1Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor1Value);
        DEBUG_SERIAL.print(F("   manipulatorMotor2Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor2Value);
        DEBUG_SERIAL.print(F("   manipulatorMotor3Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor3Value);
        DEBUG_SERIAL.print(F("   manipulatorMotor4Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor4Value);
        DEBUG_SERIAL.print(F("   description : "));
        DEBUG_SERIAL.println(manipulatorPose.description);
        DEBUG_SERIAL.println();
        turnOnLEDGreen500ms();
      } else {
        DEBUG_SERIAL.println(F("read command : there is no data"));
        DEBUG_SERIAL.println();
        turnOnLEDRed500ms();
      }
      
    } else if (currentCommandStatus == REMOVE) {
      if (receivedString.equalsIgnoreCase("help")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("remove command : help"));
        DEBUG_SERIAL.println();
        printTypeRemoveCommandParameterGuide();
        return;
      } else if (receivedString.equalsIgnoreCase("quit")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("remove command : quit"));
        DEBUG_SERIAL.println();
        currentCommandStatus = COMMAND;
        printSelectCommandGuide();
        return;
      }
      
      // 파라미터 개수 체크
      uint8_t paramCnt = 1;
      for (int i = 0 ; i < receiveBufferIdx ; i++) {
        if (receiveBuffer[i] == ',') paramCnt++;
      }
      if (paramCnt != REMOVE_COMMAND_PARAMETER_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("remove command : wrong parameters count, it needs one parameter"));
        DEBUG_SERIAL.println();
        printTypeRemoveCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      uint8_t substringFromIdx = 0;
      uint8_t substringToIdx = receivedString.length();
      uint8_t id = receivedString.substring(substringFromIdx, substringToIdx).toInt();
      if (id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("remove command : wrong id"));
        DEBUG_SERIAL.println();
        printTypeRemoveCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      RemoveManipulatorPresentPoseFromEEPROM(id);
      
      resetAllReceivedData();
      DEBUG_SERIAL.println(F("remove command : pose is removed"));
      DEBUG_SERIAL.println();
      turnOnLEDGreen500ms();
      
    } else if (currentCommandStatus == RUN) {
      if (receivedString.equalsIgnoreCase("help")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("run command : help"));
        DEBUG_SERIAL.println();
        printTypeRunCommandParameterGuide();
        return;
      } else if (receivedString.equalsIgnoreCase("quit")) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("run command : quit"));
        DEBUG_SERIAL.println();
        currentCommandStatus = COMMAND;
        printSelectCommandGuide();
        return;
      }

      // 파라미터 개수 체크
      uint8_t paramCnt = 1;
      for (int i = 0 ; i < receiveBufferIdx ; i++) {
        if (receiveBuffer[i] == ',') paramCnt++;
      }
      if (paramCnt < RUN_COMMAND_PARAMETER_MIN_CNT || paramCnt > RUN_COMMAND_PARAMETER_MAX_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("run command : wrong parameters count, it needs two or three parameters"));
        DEBUG_SERIAL.println();
        printTypeRunCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      uint8_t substringFromIdx = 0;
      uint8_t substringToIdx = receivedString.indexOf(',', substringFromIdx);
      uint8_t id = receivedString.substring(substringFromIdx, substringToIdx).toInt();
      if (id < 1 || id > MANIPULATOR_POSE_ID_MAX_CNT) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("run command : wrong id"));
        DEBUG_SERIAL.println();
        printTypeRunCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      substringFromIdx = substringToIdx+1;
      substringToIdx = receivedString.indexOf(',', substringFromIdx);
      int32_t operatingTime = receivedString.substring(substringFromIdx, substringToIdx).toInt();
      if (operatingTime <= 0 || operatingTime > LONG_MAX) {
        resetAllReceivedData();
        DEBUG_SERIAL.println(F("run command : wrong operating time"));
        DEBUG_SERIAL.println();
        printTypeRunCommandParameterGuide();
        turnOnLEDRed500ms();
        return;
      }

      float motor1Angle = -360.0;
      if (paramCnt == RUN_COMMAND_PARAMETER_MAX_CNT) {
        substringFromIdx = substringToIdx+1;
        substringToIdx = receivedString.length();

        // 0인지 체크
        bool isMotor1AngleZero = true;
        for (int i = substringFromIdx ; i < substringToIdx ; i++) {
          if (receivedString.charAt(i) == '-' || receivedString.charAt(i) == '0' || receivedString.charAt(i) == '.' || receivedString.charAt(i) == ' ') continue;
          isMotor1AngleZero = false;
        }

        if (isMotor1AngleZero) {
          motor1Angle = 0.0;
        } else {
          motor1Angle = receivedString.substring(substringFromIdx, substringToIdx).toFloat();
          if (motor1Angle == 0.0) {
            resetAllReceivedData();
            DEBUG_SERIAL.println(F("run command : wrong motor1 angle"));
            DEBUG_SERIAL.println();
            printTypeRunCommandParameterGuide();
            turnOnLEDRed500ms();
            return;
          }
          if (motor1Angle < -100.0) motor1Angle = -100.0;  // constrain
          if (motor1Angle > 100.0) motor1Angle = 100.0;
        }
      }
      
      ManipulatorPose manipulatorPose;
      if (motor1Angle == -360.0)
        manipulatorPose = RunManipulatorPoseWithPoseDataInEEPROM(id, operatingTime);
      else
        manipulatorPose = RunManipulatorPoseWithPoseDataInEEPROM(id, operatingTime, motor1Angle);
      
      resetAllReceivedData();
      
      if (manipulatorPose.isTherePoseData) {
        DEBUG_SERIAL.println(F("run command : success"));
        DEBUG_SERIAL.print(F("   id : "));
        DEBUG_SERIAL.println(manipulatorPose.id);
        DEBUG_SERIAL.print(F("   manipulatorMotor1Value : "));
        if (motor1Angle != -360.0) {
          DEBUG_SERIAL.println(map(motor1Angle, -180, 180, 0, 4095));
        } else {
          DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor1Value);
        }
        DEBUG_SERIAL.print(F("   manipulatorMotor2Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor2Value);
        DEBUG_SERIAL.print(F("   manipulatorMotor3Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor3Value);
        DEBUG_SERIAL.print(F("   manipulatorMotor4Value : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor4Value);
        DEBUG_SERIAL.print(F("   description : "));
        DEBUG_SERIAL.println(manipulatorPose.description);
        DEBUG_SERIAL.print(F("   operatingTimeMillis : "));
        DEBUG_SERIAL.println(operatingTime);
        DEBUG_SERIAL.println();
        turnOnLEDGreen500ms();
      } else {
        DEBUG_SERIAL.println(F("run command : there is no data"));
        DEBUG_SERIAL.println();
        turnOnLEDRed500ms();
      }
      
    }
  }
}

void flushDebugSerialData() {
  while (DEBUG_SERIAL.available()) {DEBUG_SERIAL.read();} // flush all received data
}

void resetAllReceivedData() {
  memset(receiveBuffer, 0, RECEIVE_BUFFER_SIZE);
  receiveBufferIdx = 0;
  flushDebugSerialData();
}

void printSelectCommandGuide() {
  DEBUG_SERIAL.println(F("type command and enter(\"\\n\")"));
  DEBUG_SERIAL.println(F("   \"on\" to turn on manipulator motor torque"));
  DEBUG_SERIAL.println(F("   \"off\" to turn off manipulator motor torque"));
  DEBUG_SERIAL.println(F("   \"list\" to show manipulator's pose list"));
  DEBUG_SERIAL.println(F("   \"save\" to save current manipulator's pose"));
  DEBUG_SERIAL.println(F("   \"read\" to read and print specific manipulator's pose"));
  DEBUG_SERIAL.println(F("   \"remove\" to remove specific manipulator's pose"));
  DEBUG_SERIAL.println(F("   \"run\" to run specific manipulator's pose"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
}

void printTypeSaveCommandParameterGuide() {
  DEBUG_SERIAL.println(F("type pose id(1~100), description and enter(\"\\n\")to save"));
  DEBUG_SERIAL.println(F("   parameters is separated by comma without space"));
  DEBUG_SERIAL.println(F("   id(uint8_t),description(char[30])"));
  DEBUG_SERIAL.println(F("or type command and enter(\"\\n\")"));
  DEBUG_SERIAL.println(F("   \"help\" to show guide message"));
  DEBUG_SERIAL.println(F("   \"quit\" to return command selection"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
}

void printTypeReadCommandParameterGuide() {
  DEBUG_SERIAL.println(F("type pose id and enter(\"\\n\")to read"));
  DEBUG_SERIAL.println(F("   id(uint8_t)"));
  DEBUG_SERIAL.println(F("or type command and enter(\"\\n\")"));
  DEBUG_SERIAL.println(F("   \"help\" to show guide message"));
  DEBUG_SERIAL.println(F("   \"quit\" to return command selection"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
}

void printTypeRemoveCommandParameterGuide() {
  DEBUG_SERIAL.println(F("type pose id and enter(\"\\n\")to remove"));
  DEBUG_SERIAL.println(F("   id(uint8_t)"));
  DEBUG_SERIAL.println(F("or type command and enter(\"\\n\")"));
  DEBUG_SERIAL.println(F("   \"help\" to show guide message"));
  DEBUG_SERIAL.println(F("   \"quit\" to return command selection"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
}

void printTypeRunCommandParameterGuide() {
  DEBUG_SERIAL.println(F("type pose id, operating time( > 0ms), motor1 angle(-100~100) and enter(\"\\n\")to run"));
  DEBUG_SERIAL.println(F("   parameters is separated by comma without space"));
  DEBUG_SERIAL.println(F("   id(uint8_t), operatingTimeMillis(int32_t), motor1Angle(float, optional)"));
  DEBUG_SERIAL.println(F("or type command and enter(\"\\n\")"));
  DEBUG_SERIAL.println(F("   \"help\" to show guide message"));
  DEBUG_SERIAL.println(F("   \"quit\" to return command selection"));
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println();
}

void turnOnLEDRed500ms() {
  setRGBLEDRed();
  delay(500);
  setRGBLEDOff();
}

void turnOnLEDGreen500ms() {
  setRGBLEDGreen();
  delay(500);
  setRGBLEDOff();
}
