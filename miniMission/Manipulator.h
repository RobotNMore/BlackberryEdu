#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "Motor.h"

//////////////  모터 구동용 변수/상수
#define ARM_DXL_ID_1            0x05 // 매니퓰레이터 1번 모터 아이디 (가장 아래)
#define ARM_DXL_ID_2            0x06 // 매니퓰레이터 2번 모터 아이디
#define ARM_DXL_ID_3            0x07 // 매니퓰레이터 3번 모터 아이디
#define ARM_DXL_ID_4            0x08 // 매니퓰레이터 4번 모터 아이디 (가장 위)

#define ARM_DXL_1_POSITION_MIN  0
#define ARM_DXL_1_POSITION_MAX  4095
#define ARM_DXL_2_POSITION_MIN  1024
#define ARM_DXL_2_POSITION_MAX  3474
#define ARM_DXL_3_POSITION_MIN  211
#define ARM_DXL_3_POSITION_MAX  2640
#define ARM_DXL_4_POSITION_MIN  722
#define ARM_DXL_4_POSITION_MAX  3161

#define ARM_DXL_1_OFFSET        0
#define ARM_DXL_2_OFFSET        238
#define ARM_DXL_3_OFFSET        799
#define ARM_DXL_4_OFFSET        0


// FinManipulatorServos 함수에서 찾을 모터 아이디들, 값이 중복 없이 정렬되어있어야 함
const uint8_t ARM_DXL_ID_CNT = 4;
const uint8_t ARM_DXL_IDS[ARM_DXL_ID_CNT] = {ARM_DXL_ID_1,
                                             ARM_DXL_ID_2,
                                             ARM_DXL_ID_3,
                                             ARM_DXL_ID_4};

// arm sync write 용 상수, 구조체 정의
const uint16_t SW_JOINT_POSITION_START_ADDR = 112; // sync write start 주소
const uint16_t SW_JOINT_POSITION_DATA_SIZE = 8; // sync write data 길이
typedef struct sw_joint_position_data{ // 모터에 데이터를 쓰기 위한 구조체 정의
  int32_t profile_velocity;
  int32_t goal_position;
} __attribute__((packed)) sw_joint_position_data_t;

// arm position control mode sync read 용 상수, 구조체 정의
const uint16_t joint_joint_position_pkt_buf_cap = 128;

const uint16_t SR_JOINT_POSITION_START_ADDR = 132; // sync read start 주소
const uint16_t SR_JOINT_POSITION_DATA_SIZE = 4; // sync read data 길이
typedef struct sr_joint_position_data{ // 모터에서 데이터를 읽기 위한 구조체 정의
  int32_t present_position;
} __attribute__((packed)) sr_joint_position_data_t;


//////////////  매니퓰레이터 계산용 변수/상수
const float d = 79.75; // 바닥에서 매니퓰레이터 2번 모터 회전축까지의 거리
const float L1 = 109.21; // 2번과 3번 모터의 회전축간 거리
const float L2 = 86.4; // 3번과 4번 모터의 회전축간 거리
const float L3 = 97.2; // 4번 모터 회전축과 그리퍼 끝부분 사이의 거리


//////////////  매니퓰레이터 포즈 저장용
#define MANIPULATOR_POSE_ID_MAX_CNT           100
#define MANIPULATOR_POSE_DATA_SIZE            40
#define MANIPULATOR_POSE_DESCRIPTION_SIZE     30

typedef struct _ManipulatorPose { // EEPROM에 저장되는 매니퓰레이터 포즈객체
  bool isTherePoseData;
  uint8_t id;
  int16_t manipulatorMotor1Value;
  int16_t manipulatorMotor2Value;
  int16_t manipulatorMotor3Value;
  int16_t manipulatorMotor4Value;
  char description[MANIPULATOR_POSE_DESCRIPTION_SIZE];
} __attribute__((packed)) ManipulatorPose;


//////////////  매니퓰레이터 함수 선언

/*
 * 매니퓰레이터의 토크를 켜는 함수
 */
void turnOnManipulatorTorque(Dynamixel2Arduino dxl);

/*
 * 매니퓰레이터의 토크를 끄는 함수
 */
void turnOffManipulatorTorque(Dynamixel2Arduino dxl);

/* 
 * 각도를 입력받아 모터의 위치 값을 반환하는 함수
 * argument : 
 *    angle : 모터의 목표 각도
 * return :
 *    대응하는 모터의 목표 위치 값. -1은 변환 실패
 */
int32_t getArmServoGoalPositionWithAngle(float angle);

/* 
 * 매니퓰레이터를 네 모터의 각도 값과 시간으로 제어하는 함수
 * argument : 
 *    a1 : 5번 모터 각도
 *    a2 : 6번 모터 각도
 *    a3 : 7번 모터 각도
 *    a4 : 8번 모터 각도
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void setManipulatorForwardMoveWithAngleForSyncWrite( Dynamixel2Arduino dxl,
                                                     float a1,
                                                     float a2,
                                                     float a3,
                                                     float a4,
                                                     int32_t operatingTime);

/* 
 * 매니퓰레이터를 네 모터의 값과 시간으로 제어하는 함수
 * argument : 
 *    mv1 : 5번 모터 값
 *    mv2 : 6번 모터 값
 *    mv3 : 7번 모터 값
 *    mv4 : 8번 모터 값
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void setManipulatorForwardMoveWithMotorValueForSyncWrite( Dynamixel2Arduino dxl,
                                                          int32_t mv1,
                                                          int32_t mv2,
                                                          int32_t mv3,
                                                          int32_t mv4,
                                                          int32_t operatingTime);

/*
 * 그리퍼의 위치와 그리퍼의 피치 각도로 4-DOF 매니퓰레이터를 제어하는 함수
 * params :
 *    x : 그리퍼 좌우 위치(mm)
 *    y : 그리퍼 전후 위치(mm)
 *    z : 그리퍼 높이(mm)
 *    angle : 그리퍼 피치 각도(degree)
 *    operatingTime : 매니퓰레이터 동작이 완료되기까지 소요될 시간
 */
void setManipulatorInverseMoveForSyncWrite( Dynamixel2Arduino dxl,
                                            int16_t x, int16_t y, int16_t z,
                                            int16_t angle,
                                            int32_t operatingTime );

/* 
 * 매니퓰레이터 모터가 통신 가능한지 확인하고 초기화 하는 함수
 * return :
 *    true  : 통신 체크 및 초기화 성공
 *    false : 통신 체크 및 초기화 실패
 */                        
bool InitManipulator(Dynamixel2Arduino dxl);

/*
 * ARM_DXL_IDS 배열에 있는 아이디의 모터들이 모두 통신 가능한지
 * 확인하는 함수
 * return :
 *    true  : 통신 체크 성공
 *    false : 통신 체크 실패
 */
bool FindManipulatorServos(Dynamixel2Arduino dxl);

/*
 * 매니퓰레이터 포즈 리스트를 출력
 */
void PrintManipulatorPoseListFromEEPROM();

/*
 * 매니퓰레이터 네 모터의 위치를 읽어서 EEPROM에 저장
 * params :
 *    id : 저장할 포즈의 아이디
 *    description : 포즈에 대한 설명(최대 30byte)
 */
void WriteManipulatorPresentPoseToEEPROM(Dynamixel2Arduino dxl, uint8_t id, String description);

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 반환
 * return :
 *    ManipulatorPose 객체 : 해당 아이디를 가진 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id);

/*
 * EEPROM 에서 지정된 id의 매니퓰레이터 포즈 데이터를 삭제
 * isTherePoseData만 false로 변경하기 때문에 데이터가 실제로 지워지지는 않음
 * params :
 *    id : 삭제할 포즈의 아이디
 */
void RemoveManipulatorPresentPoseFromEEPROM(uint8_t id);

/*
 * 매니퓰레이터 포즈 데이터를 EEPROM에서 읽어 실행
 * params :
 *    id : 실행할 포즈의 아이디
 *    operatingTimeMillis : 포즈 실행에 사용할 시간
 *    motor1Angle(-100~100) : 1번 모터 회전 각도(기본 값은 -360.0이며, 각도 값을 주지 않으면 포즈 데이터의 값을 그대로 사용)
 *                            과한 회전을 방지하기 위해 범위를 -100~100도로 하였음
 * return :
 *    ManipulatorPose 객체 : 실행한 ManipulatorPose 객체를 반환
 *                          isTherePoseData로 데이터가 유효한지 확인한 후 사용해야 함
 */
ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM(Dynamixel2Arduino dxl, uint8_t id, int32_t operatingTimeMillis, float motor1Angle = -360.0);

#endif
