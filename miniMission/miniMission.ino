//#define STOP_TO_COLOR_CALIBRATION

#include "Debug.h"
#include "Pixy.h"
#include "Manipulator.h"
#include "Mobilebase.h"
#include "PSD.h"
#include "Gripper.h"

#define PIXY2_X_SETPOINT                      172 // 카메라 x위치 목표값. 실험 후 보정하여 사용할 수 있음
#define PIXY2_Y_SETPOINT                      107 // 카메라 y위치 기준값

#define PIXY_TOLERANCE                        2
#define PSD_TOLERANCE                         2
#define PIXY_CONTROL_RATIO                    0.3
#define PSD_CONTROL_RATIO                     0.04

#define PSD_FL_CORRECTION                     -12  // 전방 PSD 두 개가 같은 거리에서 비슷한 값을 가지게 하는 보정값

#define OBSTACLE_FRONT_PSD_SET_POINT          525
#define OBSTACLE_LEFT_PSD_SET_POINT           418
#define MISSION_FRONT_PSD_SET_POINT           310
#define MISSION_LEFT_PSD_SET_POINT            595

//////////////  Pixy2 카메라
Pixy2SPI_SS pixy;        // SPI 통신하는 Pixy2 객체 생성

//////////////  모터
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//////////////  미션용 상수 및 변수
enum ManipulatorPoseID {
  INITIAL_AND_MISSION_INSTRUCTION = 1,
  STORAGE,
  PRE_GRIP_UPPER_BLOCK,
  GRIP_UPPER_BLOCK,
  PRE_GRIP_LOWER_BLOCK,
  GRIP_LOWER_BLOCK
};
#define MANIPULATOR_MISSION_FULFILLMENT_POSE_START_ID  6

bool haveFoundBlock = false;
const uint8_t MISSION_BLOCK_CNT = 2;
//const uint8_t BLOCK_SIG_MAP = 0b00000000;  // 0번 비트는 sig1, 1번 비트는 sig2... 관심가지는 시그니처의 비트를 1로 set하면 해당 색상만 검색됨
//uint8_t targetBlockSigmap = 0x01;
uint8_t targetBlockSigmaps[MISSION_BLOCK_CNT] = {0x01, 0x02};
uint8_t goalPositions[MISSION_BLOCK_CNT] = {7, 8};
                                             
void setup() {
  InitDebug(); // 디버그를 사용하는 다른 것들보다 먼저 해주어야 함
  InitMotorCommunication(dxl);  // InitManipulator, InitMobileBase보다 먼저 해 주어야 함
  while(!InitManipulator(dxl)) {}
  while(!InitMobilebase(dxl)) {}
  
  InitPSD();
  InitPixy(pixy);  // Camera, Gripper를 사용하기 위해 해 주어야 함
  pixy.setLamp(0, 0); // 카메라 램프 끄기
  OpenGripper(pixy);

//  int16_t slTest;  // 좌측면 PSD 거리 테스트
//  while(1) {
//    GetValueFromSideLeftPSDSensor(&slTest);
//    Serial.println(slTest);
//    delay(100);
//  }

  DEBUG_SERIAL.println("run");

//  // 초기화. 미션지시 Zone을 보도록 매니퓰레이터 움직이기
//  OpenGripper(pixy);
//  RunManipulatorPoseWithPoseDataInEEPROM(dxl, INITIAL_AND_MISSION_INSTRUCTION, 2000);
//  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );  // 정지
//  delay(3000);

  // 물류미션물품적재함을 보도록 매니퓰레이터 움직이기
  // todo 이게 지금 그리퍼가 경기장의 시작 선 밖으로 나가는데 이걸 처리를 해야하는지.. 규정 확인 필요
  RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 1000, 0.0);
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);  // 정지
  delay(1000);

  // 전방 PSD 값 맞춰서 전방으로 이동하기
  int16_t flPSDValue1, frPSDValue1;
  while(1) {
    GetValueFromFrontPSDSensors(&flPSDValue1, &frPSDValue1);
    if (!DriveForwardUntilDistanceWithTwoSensors(dxl, flPSDValue1+PSD_FL_CORRECTION - OBSTACLE_FRONT_PSD_SET_POINT,
                                                 frPSDValue1 - OBSTACLE_FRONT_PSD_SET_POINT, PSD_TOLERANCE)) break;
  }

  // 좌측 PSD 값 맞춰서 좌측으로 이동하기
  int16_t slPSDValue2;
  while(1) {
    GetValueFromSideLeftPSDSensor(&slPSDValue2);
    if (!DriveUntilDistanceWithOneSensor(dxl, slPSDValue2 - OBSTACLE_LEFT_PSD_SET_POINT, PSD_TOLERANCE, DRIVE_DIRECTION_LEFT)) break;
  }

//  // 블록 색상 확인하기 -> 처음에 미션 인식 구역을 인식하지 않고 지나가도록 주석처리 함
//  pixy.ccc.getBlocks(true, BLOCK_SIG_MAP);
//
//  DEBUG_SERIAL.print("Detected ");  // 블록이 몇개 인식됐는지 출력
//  DEBUG_SERIAL.println(pixy.ccc.numBlocks);
//  
//  if (pixy.ccc.numBlocks) {
//    targetBlockSigmap = (1 << pixy.ccc.blocks[0].m_signature-1);  // 가장 크게 인식 된 첫 번째 블록만 저장
//    
//    for (int i = 0 ; i < pixy.ccc.numBlocks ; i++) {  // 인식된 전체 블록의 정보 출력
//      DEBUG_SERIAL.print("  instruction block ");
//      DEBUG_SERIAL.print(i);
//      DEBUG_SERIAL.print(": ");
//      pixy.ccc.blocks[i].print();  // Serial로만 출력됨
//    }
//  }

//  // 물류미션물품적재함을 보도록 매니퓰레이터 움직이기. 초기화 시에 미리 해당 위치를 보게 해 놓아서 생략
//  RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 1000, 0.0);
//  delay(1000);

  // 정해진 거리만큼 전방으로 이동하기
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, 800.0);
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  // 정해진 거리만큼 우측으로 이동하기
  DriveDistanceAndMmPerSecAndDirection(dxl, 30.0, DRIVE_DIRECTION_RIGHT);
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

//  // 정해진 거리만큼 대각선으로 이동하기 -> 대각선 이동은 슬립이 심한 편이라 전방과 우측 이동으로 분리해서 위 코드에서 동작하게 했음
//  DriveXYDistanceAndMmPerSec(dxl, 60.0, 350.0);
//  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  ChangeMobilebaseMode2VelocityControlMode(dxl);
  
  for (int missionIdx = 0 ; missionIdx < MISSION_BLOCK_CNT ; missionIdx++) {  // 미션블록 개수만큼 반복
  
    // 좌측, 전방 우측 PSD 값 맞춰서 이동하기
    int16_t slPSDValue3, frPSDValue3;
    while(1) {
      GetValueFromSideLeftPSDSensor(&slPSDValue3);
      GetValueFromFrontRightPSDSensor(&frPSDValue3);
      if (!LocateWithTwoSensors(dxl, slPSDValue3 - MISSION_LEFT_PSD_SET_POINT, frPSDValue3 - MISSION_FRONT_PSD_SET_POINT,
                                PSD_TOLERANCE, PSD_TOLERANCE, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                DRIVE_DIRECTION_LEFT)) break;
    }

    pixy.setLamp(1, 1); // 카메라 램프 켜기
    delay(500);

#ifdef STOP_TO_COLOR_CALIBRATION
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);  // 정지
    while(1) {delay(1);}
#endif
    
    // 오른쪽 진행: 전방 우측 PSD에 장애물이 인식됐는지 확인하면서 정면에 미션블록이 있는지 확인
    int16_t slPSDValue4, flPSDValue4, frPSDValue4;
    while(!haveFoundBlock) {
      // 좌측 PSD 값 업데이트
      GetValueFromSideLeftPSDSensor(&slPSDValue4);
      if (slPSDValue4 < 160) {  // 테이블 오른쪽 끝
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);  // 정지
        break;
      }

      // 정면 PSD들 값 업데이트
      GetValueFromFrontPSDSensors(&flPSDValue4, &frPSDValue4);
      // 카메라에 인식된 미션블록 위치에 따라 x위치 오차 계산
      pixy.ccc.getBlocks(true, targetBlockSigmaps[missionIdx]);
      int16_t blockXError = (pixy.ccc.numBlocks/* && (pixy.ccc.blocks[0].m_x > 126 && pixy.ccc.blocks[0].m_x < 189) *//* cammera ROI */
                             ? pixy.ccc.blocks[0].m_x : PIXY_CCC_X_MAX) - PIXY2_X_SETPOINT;
      // PSD사용한 y위치 오차, 회전 오차 변수
      int16_t yPosError, rotationError;
      bool inControl = true;  // 모바일베이스 제어중 or 제어완료 상태 플래그

      // 좌측 센서에 인식되는 거리에 따라 어떻게 주행할건지, 정면 센서를 어떻게 사용할 것인지 결정
      if (slPSDValue4 < 190) {  // 회전보정 가능구간 끝. 좌측 센서만으로 주행
        yPosError = flPSDValue4+PSD_FL_CORRECTION - MISSION_FRONT_PSD_SET_POINT;
        inControl = LocateWithTwoSensors(dxl,blockXError, yPosError,
                                         PIXY_TOLERANCE, PSD_TOLERANCE, PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                         DRIVE_DIRECTION_LEFT);
      } else if (slPSDValue4 < 245) {  // 회전보정 가능구간 시작. 회전보정
        // 정면 장애물과 거리 맞추면서 회전 보정하면서 우로 진행
        yPosError = (flPSDValue4+PSD_FL_CORRECTION+frPSDValue4)/2 - MISSION_FRONT_PSD_SET_POINT;
        rotationError = frPSDValue4 - (flPSDValue4+PSD_FL_CORRECTION);
        inControl = DriveWithPositionAndRotationErrors(dxl, blockXError, yPosError, rotationError,
                                                       PIXY_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                                                       PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                                       DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW);
      } else {  // 테이블 왼쪽. 전방우측 센서만으로 전방거리 보정
        yPosError = frPSDValue4 - MISSION_FRONT_PSD_SET_POINT;
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                                         PIXY_TOLERANCE, PSD_TOLERANCE, PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                         DRIVE_DIRECTION_LEFT);
      }
      
      if (!inControl) {
        haveFoundBlock = true;
      }
    }

    // 왼쪽 진행
    while(!haveFoundBlock) {
      GetValueFromSideLeftPSDSensor(&slPSDValue4);
      if (slPSDValue4 > 635) {  // 테이블 왼쪽 끝. 정지
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);  // 정지
        break;
      }
      
      GetValueFromFrontPSDSensors(&flPSDValue4, &frPSDValue4);
      pixy.ccc.getBlocks(true, targetBlockSigmaps[missionIdx]);
      int16_t blockXError = (pixy.ccc.numBlocks/* && (pixy.ccc.blocks[0].m_x > 126 && pixy.ccc.blocks[0].m_x < 189) *//* camera ROI */
                             ? pixy.ccc.blocks[0].m_x : PIXY_CCC_X_MIN) - PIXY2_X_SETPOINT;
      int16_t yPosError, rotationError;
      bool inControl = true;
      
      // 정면에 미션블록이 있는지 확인
      
      if (slPSDValue4 < 190) {  // 회전보정 가능구간 끝. 좌측 센서만으로 주행
        yPosError = flPSDValue4+PSD_FL_CORRECTION - MISSION_FRONT_PSD_SET_POINT;
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                                         PIXY_TOLERANCE, PSD_TOLERANCE, PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                         DRIVE_DIRECTION_LEFT);
      } else if (slPSDValue4 < 245) {  // 회전보정 가능구간 시작. 회전보정
        // 정면 장애물과 거리 맞추면서 회전 보정하면서 우로 진행
        yPosError = (flPSDValue4+PSD_FL_CORRECTION+frPSDValue4)/2 - MISSION_FRONT_PSD_SET_POINT;
        rotationError = frPSDValue4 - (flPSDValue4+PSD_FL_CORRECTION);
        inControl = DriveWithPositionAndRotationErrors(dxl, blockXError, yPosError, rotationError,
                                                       PIXY_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                                                       PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                                       DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW);
      } else {  // 테이블 왼쪽. 우측 센서만으로 주행
        yPosError = frPSDValue4 - MISSION_FRONT_PSD_SET_POINT;
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                                         PIXY_TOLERANCE, PSD_TOLERANCE, PIXY_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                         DRIVE_DIRECTION_LEFT);
      }
      
      if (!inControl) {
        haveFoundBlock = true;
      }
    }

    // 앞선 과정에서 블록을 찾았다면 잡으러가기
    if (haveFoundBlock) {
      // 블록을 잡도록 매니퓰레이터 움직이기
      // Y기준값을 사용해 블록이 위에 있는지 아래에 있는지 판단
      if (pixy.ccc.blocks[0].m_y < PIXY2_Y_SETPOINT) { // 블록이 위층에 있음
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, PRE_GRIP_UPPER_BLOCK, 1000, 0.0);
        delay(1200);
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, GRIP_UPPER_BLOCK, 1000, 0.0);
        delay(1500);
      } else { // 블록이 아래층에 있음
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, PRE_GRIP_LOWER_BLOCK, 1000, 0.0);
        delay(1200);
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, GRIP_LOWER_BLOCK, 1000, 0.0);
        delay(1500);
      }
    
      // 그리퍼 닫기
      CloseGripper(pixy);
      delay(500);

      pixy.setLamp(0, 0); // 카메라 램프 끄기
      
      // 좌측으로 이동할 수 있도록 매니퓰레이터를 로봇 쪽으로 움직이기
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 1000, 0.0);
      delay(1500);
    
      // 좌측 PSD 값 맞춰서 좌측으로 블록 넣으러 이동하기
      while(1) {
        GetValueFromSideLeftPSDSensor(&slPSDValue4);
        GetValueFromFrontPSDSensors(&flPSDValue4, &frPSDValue4);
        int16_t xPosError = slPSDValue4 - MISSION_LEFT_PSD_SET_POINT;
        int16_t yPosError, rotationError;
        bool inControl = true;
        
        if (slPSDValue4 < 190) {  // 회전보정 가능구간 끝. 좌측 센서만으로 주행
          yPosError = flPSDValue4+PSD_FL_CORRECTION - MISSION_FRONT_PSD_SET_POINT;
          inControl = LocateWithTwoSensors(dxl, xPosError, yPosError,
                                           PSD_TOLERANCE, PSD_TOLERANCE, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                           DRIVE_DIRECTION_LEFT);
        } else if (slPSDValue4 < 245) {  // 회전보정 가능구간 시작. 회전보정
          // 정면 장애물과 거리 맞추면서 회전 보정하면서 우로 진행
          yPosError = (flPSDValue4+PSD_FL_CORRECTION+frPSDValue4)/2 - MISSION_FRONT_PSD_SET_POINT;
          rotationError = frPSDValue4 - (flPSDValue4+PSD_FL_CORRECTION);
          inControl = DriveWithPositionAndRotationErrors(dxl, xPosError, yPosError, rotationError,
                                                         PSD_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                                                         PSD_CONTROL_RATIO, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                                         DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW);
        } else {  // 테이블 왼쪽. 우측 센서만으로 주행
          yPosError = frPSDValue4 - MISSION_FRONT_PSD_SET_POINT;
          inControl = LocateWithTwoSensors(dxl, xPosError, yPosError,
                                           PSD_TOLERANCE, PSD_TOLERANCE, PSD_CONTROL_RATIO, PSD_CONTROL_RATIO,
                                           DRIVE_DIRECTION_LEFT);
        }

        if (!inControl) {
          break;
        }
      }
    
      // 미션수행 Zone 정해진 칸으로 매니퓰레이터 움직이기
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 800, -90.0);
      delay(1000);
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, MANIPULATOR_MISSION_FULFILLMENT_POSE_START_ID + goalPositions[missionIdx], 1000);
      delay(1300);
    
      // 그리퍼 열기
      OpenGripper(pixy);
      delay(500);
      
      // 매니퓰레이터를 로봇 쪽으로 움직이기
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 1000, 0.0);
      delay(1500);

      haveFoundBlock = false;
    }
  }


  // 도착 위치까지 후진
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  DriveDistanceAndMmPerSecAndDirection(dxl, 1280.0, DRIVE_DIRECTION_BACKWARD);

  
  // END
  while(1) { delay(1000); }
}

void loop() {
}
