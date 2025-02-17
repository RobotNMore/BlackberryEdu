#define DEBUG
#define DEBUG_SERIAL    Serial

#include "Pins.h"
#include "Motor.h"
#include "Manipulator.h"

#include <OneButton.h>
OneButton leftButton(SW1_PIN, true);
OneButton rightButton(SW2_PIN, true);

//////////////  매니퓰레이터 계산용 변수/상수
#define pi      3.141592
#define DTR(x)  (x)*(pi/180) // degree to radian


#define INITIAL_MOTOR_1_ANGLE         0
#define INITIAL_MOTOR_2_ANGLE         45
#define INITIAL_MOTOR_3_ANGLE         -90
#define INITIAL_MOTOR_4_ANGLE         -45
#define INITIAL_POSE_OPERATING_TIME   2000

int16_t m1Value = 0;
int16_t m2Value = 0;
int16_t m3Value = 0;
int16_t m4Value = 0;

float m1Angle = 0.0;
float m2Angle = 0.0;
float m3Angle = 0.0;
float m4Angle = 0.0;

float theta1 = 0.0;
float theta2 = 0.0;
float theta3 = 0.0;
float theta4 = 0.0;

float X = 0; // 모터 각도에 따른 그리퍼의 위치 결과 값
float Y = 0;
float Z = 0;
float Yd = 0; // 매니퓰레이터 단면에서 수평방향 길이

void setup() {
  Serial.begin(115200);

  leftButton.attachClick(calcAndPrintCurrentPoseData);
  rightButton.attachClick(toggleArmTorque);  // todo 해당 버튼 기획필요

  initMotorCommunication();  // initManipulator, initMobileBase보다 먼저 해 주어야 함
  while(!initManipulator()) {}
  
  SetManipulatorForwardMoveWithAngleForSyncWrite( INITIAL_MOTOR_1_ANGLE,
                                                  INITIAL_MOTOR_2_ANGLE,
                                                  INITIAL_MOTOR_3_ANGLE,
                                                  INITIAL_MOTOR_4_ANGLE,
                                                  INITIAL_POSE_OPERATING_TIME );
   delay(2000);
   Serial.println("start");
}

void loop() {
  leftButton.tick();
  rightButton.tick();
}

void toggleArmTorque() {
  if (dxl.readControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[0]) == TORQUE_ON) {
    Serial.println("manipulator torque off");
    turnOffManipulatorTorque();
  } else {
    Serial.println("manipulator torque on");
    turnOnManipulatorTorque();
  }
}

void calcAndPrintCurrentPoseData() {
  readCurrentPoseValue();
  calcCurrentPoseData();
  printCalculatedPoseData();
}

void readCurrentPoseValue() {
  m1Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  m2Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  m3Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  m4Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
}

void calcCurrentPoseData() {
  m1Angle = -180+((360.0*m1Value)/4096.0);
  m2Angle = -180+((360.0*m2Value)/4096.0);
  m3Angle = -180+((360.0*m3Value)/4096.0);
  m4Angle = -180+((360.0*m4Value)/4096.0);
  
  // 모터각도를 그리퍼 위치 계산을 위한 세타값으로 변환
  theta1 = 90 - m1Angle;
  theta2 = 90 + m2Angle;
  theta3 = m3Angle;
  theta4 = m4Angle;

  // 좌표 계산
  Yd = L1*cos(DTR(theta2))
       + L2*cos(DTR(theta2) + DTR(theta3))
       + L3*cos(DTR(theta2) + DTR(theta3) + DTR(theta4));
       
  Z = L1*sin(DTR(theta2))
      + L2*sin(DTR(theta2) + DTR(theta3))
      + L3*sin(DTR(theta2) + DTR(theta3) + DTR(theta4))
      + d;
      
  X = Yd*cos(DTR(theta1));
  
  Y = Yd*sin(DTR(theta1));
}

void printCalculatedPoseData() {
  Serial.println("---------------------------------------------\n");
  Serial.print("manipulator motor1 value : ");
  Serial.println(m1Value);
  Serial.print("manipulator motor2 angle : ");
  Serial.println(m2Value);
  Serial.print("manipulator motor3 angle : ");
  Serial.println(m3Value);
  Serial.print("manipulator motor4 angle : ");
  Serial.println(m4Value);
  Serial.println();
  Serial.println("---------------------------------------------\n");
  Serial.print("manipulator motor1 angle : ");
  Serial.println(m1Angle);
  Serial.print("manipulator motor2 angle : ");
  Serial.println(m2Angle);
  Serial.print("manipulator motor3 angle : ");
  Serial.println(m3Angle);
  Serial.print("manipulator motor4 angle : ");
  Serial.println(m4Angle);
  Serial.println();
  Serial.println("---------------------------------------------\n");
  Serial.print("theta1 : "); // 매니퓰레이터 모터 1번의 각도
  Serial.println(theta1);
  Serial.print("theta2 : "); // 매니퓰레이터 모터 2번의 지면과의 각도
  Serial.println(theta2);
  Serial.print("theta3 : "); // 매니퓰레이터 모터 3번의 각도
  Serial.println(theta3);
  Serial.print("theta4 : "); // 매니퓰레이터 모터 4번의 각도
  Serial.println(theta4);
  Serial.println();
  Serial.println("---------------------------------------------\n");
  Serial.print("gripper position yd : ");
  Serial.println(Yd);
  Serial.println();
  Serial.print("gripper position x : ");
  Serial.println(X);
  Serial.print("gripper position y : ");
  Serial.println(Y);
  Serial.print("gripper position z : ");
  Serial.println(Z);
  Serial.println();
  Serial.println("---------------------------------------------\n");

  Serial.println("\n");
}
