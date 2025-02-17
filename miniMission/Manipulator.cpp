#include "Arduino.h"
#include "Debug.h"
#include "Manipulator.h"
#include <EEPROM.h>


// arm sync write 용 객체 인스턴스화
sw_joint_position_data_t sw_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_arm_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_joint_position_xels_sw[ARM_DXL_ID_CNT];

// arm position control mode sync read 용 객체 인스턴스화
uint8_t joint_position_pkt_buf[joint_joint_position_pkt_buf_cap];
sr_joint_position_data_t sr_joint_position_data[ARM_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_arm_position_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_joint_position_xels_sr[ARM_DXL_ID_CNT];

//////////////  매니퓰레이터 함수 정의

void turnOnManipulatorTorque(Dynamixel2Arduino dxl) {
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON);
  }
}

void turnOffManipulatorTorque(Dynamixel2Arduino dxl) {
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF);
  }
}

int32_t getArmServoGoalPositionWithAngle(Dynamixel2Arduino dxl, float angle) {
  if (!isnan(angle)) {
    // 입력받은 angle 값을 -180~180 범위 내의 값으로 제한하고
    // 0~4095 범위로 매핑
    return (int32_t)map(constrain((int32_t)angle, -180, 180),
                        -180, 180, 0, 4095);
  } else {
    return (int32_t)-1;
  }
}

void setManipulatorForwardMoveWithAngleForSyncWrite( Dynamixel2Arduino dxl,
                                                     float a1,
                                                     float a2,
                                                     float a3,
                                                     float a4,
                                                     int32_t operatingTime) {
  int32_t motor1GoalPosition = getArmServoGoalPositionWithAngle( a1 );
  int32_t motor2GoalPosition = getArmServoGoalPositionWithAngle( a2 );
  int32_t motor3GoalPosition = getArmServoGoalPositionWithAngle( a3 );
  int32_t motor4GoalPosition = getArmServoGoalPositionWithAngle( a4 );
  
  if (motor1GoalPosition != -1
      && motor2GoalPosition != -1
      && motor3GoalPosition != -1
      && motor4GoalPosition != -1) {
    setManipulatorForwardMoveWithMotorValueForSyncWrite( dxl,
                                                         motor1GoalPosition,
                                                         motor2GoalPosition,
                                                         motor3GoalPosition,
                                                         motor4GoalPosition,
                                                         operatingTime);
  }
}

void setManipulatorForwardMoveWithMotorValueForSyncWrite( Dynamixel2Arduino dxl,
                                                          int32_t mv1,
                                                          int32_t mv2,
                                                          int32_t mv3,
                                                          int32_t mv4,
                                                          int32_t operatingTime) {
                                                            
  sw_joint_position_data[0].goal_position = constrain(mv1, ARM_DXL_1_POSITION_MIN, ARM_DXL_1_POSITION_MAX);
  sw_joint_position_data[0].profile_velocity = operatingTime;
  sw_joint_position_data[1].goal_position = constrain(mv2, ARM_DXL_2_POSITION_MIN, ARM_DXL_2_POSITION_MAX);
  sw_joint_position_data[1].profile_velocity = operatingTime;
  sw_joint_position_data[2].goal_position = constrain(mv3, ARM_DXL_3_POSITION_MIN, ARM_DXL_3_POSITION_MAX);
  sw_joint_position_data[2].profile_velocity = operatingTime;
  sw_joint_position_data[3].goal_position = constrain(mv4, ARM_DXL_4_POSITION_MIN, ARM_DXL_4_POSITION_MAX);
  sw_joint_position_data[3].profile_velocity = operatingTime;
  
  sw_arm_position_infos.is_info_changed = true;
  
  while(!dxl.syncWrite(&sw_arm_position_infos)) {}
}

void setManipulatorInverseMoveForSyncWrite( Dynamixel2Arduino dxl,
                                            int16_t x, int16_t y, int16_t z,
                                            int16_t angle,
                                            int32_t operatingTime ) {
  int16_t zFromMotor2 = z - d; // 계산의 편의를 위해 2번 모터 관절과 지면 사이의 거리를
                               // 무시한 z축(위/아래) 거리
  // 매니퓰레이터를 측면에서 봤을 때 전진 거리 yd
  float yd = sqrt((int32_t)x*x + (int32_t)y*y)*(y/abs(y));
  // 매니퓰레이터를 측면에서 봤을 때 2번째 링크가 끝나는 점의 위치 (y2, z2)
  float y2 = yd - L3*cos(radians(angle));
  float z2 = zFromMotor2 - L3*sin(radians(angle));
  
  // 세타1을 구하기 위해 사용되는 변수
  float k1 = L1 + L2*(y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2);
  float k2 = L2*-1*sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2));
  float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0; // 모터별 세타 변수

  theta1 = degrees(atan2(y, x)); // atan2는 -pi~pi 범위로 결과를 반환 함
  theta2 = degrees(atan2(z2, y2)) - degrees(atan2(k2, k1));
  theta3 = degrees(atan2(-1*sqrt(1 - pow((y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2), 2)),
                     (y2*y2 + z2*z2 - L1*L1 - L2*L2)/(2*L1*L2)));
  theta4 = angle - theta2 - theta3;

  float a1 = 90 - theta1; // 세타 값을 모터 제어를 위한 각도로 변환
  if (a1 > 180) // 범위를 -180~180으로 변환
    a1 -= 360;
  float a2 = -90 + theta2;
  if (a2 < -180)
    a2 += 360;
  float a3 = theta3;
  float a4 = theta4;

  // 계산된 모터 각도로 매니퓰레이터 동작 준비
  setManipulatorForwardMoveWithAngleForSyncWrite( dxl, a1, a2, a3, a4, operatingTime);

#ifdef DEBUG
  DEBUG_SERIAL.println("Result");
  DEBUG_SERIAL.print("    theta1 = ");
  DEBUG_SERIAL.println(theta1);
  DEBUG_SERIAL.print("    theta2 = ");
  DEBUG_SERIAL.println(theta2);
  DEBUG_SERIAL.print("    theta3 = ");
  DEBUG_SERIAL.println(theta3);
  DEBUG_SERIAL.print("    theta4 = ");
  DEBUG_SERIAL.println(theta4);
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println("Result motor angle");
  DEBUG_SERIAL.print("    m1angle = ");
  DEBUG_SERIAL.println(a1);
  DEBUG_SERIAL.print("    m2angle = ");
  DEBUG_SERIAL.println(a2);
  DEBUG_SERIAL.print("    m3angle = ");
  DEBUG_SERIAL.println(a3);
  DEBUG_SERIAL.print("    m4angle = ");
  DEBUG_SERIAL.println(a4);
#endif
}

bool InitManipulator(Dynamixel2Arduino dxl) {
  // 매니퓰레이터 모터가 모두 있는지 확인
  if (!FindManipulatorServos(dxl)) return false;
  
  for (int i = 0 ; i < ARM_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_OFF);
  
    // 드라이브 모드 설정
    // 방향은 Reverse Mode, Profile Configuration은 Time-based Profile
    dxl.writeControlTableItem(DRIVE_MODE, ARM_DXL_IDS[i],
                              REVERSE_MODE + TIME_BASED_PROFILE);
    // 오퍼레이팅 모드를 위치 제어 모드로 설정
    dxl.writeControlTableItem(OPERATING_MODE, ARM_DXL_IDS[i], POSITION_CONTROL_MODE);

    switch (ARM_DXL_IDS[i]) { // 모터에 따라 position limit, homing offset 설정
      case ARM_DXL_ID_1:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT,
                                  ARM_DXL_IDS[i], ARM_DXL_1_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET,
                                  ARM_DXL_IDS[i], ARM_DXL_1_OFFSET);
        break;
      case ARM_DXL_ID_2:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_2_OFFSET);
        break;
      case ARM_DXL_ID_3:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_3_OFFSET);
        break;
      case ARM_DXL_ID_4:
        dxl.writeControlTableItem(MIN_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MIN);
        dxl.writeControlTableItem(MAX_POSITION_LIMIT, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_POSITION_MAX);
        dxl.writeControlTableItem(HOMING_OFFSET, 
                                  ARM_DXL_IDS[i], ARM_DXL_4_OFFSET);
        break;
    }

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, ARM_DXL_IDS[i], TORQUE_ON);
  }
  
  // manipulator sync write 준비
  sw_arm_position_infos.packet.p_buf = nullptr; // nullptr을 전달하면 내부 버퍼를 사용
  sw_arm_position_infos.packet.is_completed = false; // false로 초기화
  sw_arm_position_infos.addr = SW_JOINT_POSITION_START_ADDR; // 컨트롤 테이블에 sync write를 시작주소
  sw_arm_position_infos.addr_length = SW_JOINT_POSITION_DATA_SIZE; // sync write하는 데이터 길이
  sw_arm_position_infos.p_xels = info_joint_position_xels_sw; // sync write 할 모터 정보
  sw_arm_position_infos.xel_count = 0; // sync write 할 모터 개수

  sw_joint_position_data[0].profile_velocity = 1000; // 모터에 write 할 데이터 초기화
  sw_joint_position_data[0].goal_position = 2048;
  sw_joint_position_data[1].profile_velocity = 1000;
  sw_joint_position_data[1].goal_position = 2048;
  sw_joint_position_data[2].profile_velocity = 1000;
  sw_joint_position_data[2].goal_position = 2048;
  sw_joint_position_data[3].profile_velocity = 1000;
  sw_joint_position_data[3].goal_position = 2048;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_joint_position_xels_sw[i].id = ARM_DXL_IDS[i];
    info_joint_position_xels_sw[i].p_data = (uint8_t*)&sw_joint_position_data[i];
    sw_arm_position_infos.xel_count++;
  }
  
  sw_arm_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // arm position control mode sync read 준비
  sr_arm_position_infos.packet.buf_capacity = joint_joint_position_pkt_buf_cap;
  sr_arm_position_infos.packet.p_buf = joint_position_pkt_buf;
  sr_arm_position_infos.packet.is_completed = false;
  sr_arm_position_infos.addr = SR_JOINT_POSITION_START_ADDR;
  sr_arm_position_infos.addr_length = SR_JOINT_POSITION_DATA_SIZE;
  sr_arm_position_infos.p_xels = info_joint_position_xels_sr;
  sr_arm_position_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < ARM_DXL_ID_CNT; i++) {
    info_joint_position_xels_sr[i].id = ARM_DXL_IDS[i];
    info_joint_position_xels_sr[i].p_recv_buf = (uint8_t*)&sr_joint_position_data[i];
    sr_arm_position_infos.xel_count++;
  }
  sr_arm_position_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  return true;
}

bool FindManipulatorServos(Dynamixel2Arduino dxl) {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= ARM_DXL_ID_CNT) {
      uint8_t arm_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (ARM_DXL_IDS[arm_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          arm_dxl_ids_idx ++;

          if (arm_dxl_ids_idx
              == sizeof(ARM_DXL_IDS)/sizeof(uint8_t)) {
            // 찾으려는 모터를 모두 찾은 경우
            break;
          }
        } else {
          if (ids_pinged_idx == count_pinged) {
             // 통신가능한 모터가 더이상 없는 경우
             is_each_motor_found = false;
             break;
          }
        }
      }
      
      if (!is_each_motor_found) {
#ifdef DEBUG
        DEBUG_SERIAL.print("Motor IDs does not match : ");
        DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      }
    } else {
#ifdef DEBUG
      DEBUG_SERIAL.print("Motor count does not match : ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      is_each_motor_found = false;
    }
  } else{
#ifdef DEBUG
    DEBUG_SERIAL.print("Broadcast returned no items : ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    is_each_motor_found = false;
  }
  return is_each_motor_found;
}

void PrintManipulatorPoseListFromEEPROM() {
  for (int i = 1 ; i <= MANIPULATOR_POSE_ID_MAX_CNT ; i++) {
      ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM(i);
      if (manipulatorPose.isTherePoseData) {
        DEBUG_SERIAL.print(manipulatorPose.id);
        DEBUG_SERIAL.print(F("."));
        DEBUG_SERIAL.println(manipulatorPose.description);
        DEBUG_SERIAL.print(F("m1 : "));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor1Value);
        DEBUG_SERIAL.print(F(", m2 : "));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor2Value);
        DEBUG_SERIAL.print(F(", m3 : "));
        DEBUG_SERIAL.print(manipulatorPose.manipulatorMotor3Value);
        DEBUG_SERIAL.print(F(", m4 : "));
        DEBUG_SERIAL.println(manipulatorPose.manipulatorMotor4Value);
        DEBUG_SERIAL.println();
      }
  }
  DEBUG_SERIAL.println(F("fin"));
  DEBUG_SERIAL.println();
}

void WriteManipulatorPresentPoseToEEPROM(Dynamixel2Arduino dxl, uint8_t id, String description) {
  ManipulatorPose manipulatorPose;
  manipulatorPose.isTherePoseData = true;
  manipulatorPose.id = id;
  manipulatorPose.manipulatorMotor1Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[0]);
  manipulatorPose.manipulatorMotor2Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[1]);
  manipulatorPose.manipulatorMotor3Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[2]);
  manipulatorPose.manipulatorMotor4Value = dxl.readControlTableItem(PRESENT_POSITION, ARM_DXL_IDS[3]);
  description.toCharArray(manipulatorPose.description, description.length());

  EEPROM.put(MANIPULATOR_POSE_DATA_SIZE*(id-1), manipulatorPose);
}

ManipulatorPose ReadManipulatorPresentPoseToEEPROM(uint8_t id) {
  ManipulatorPose manipulatorPose;
  manipulatorPose.isTherePoseData = false;

  if (id >= 1 && id <= MANIPULATOR_POSE_ID_MAX_CNT)
    EEPROM.get(MANIPULATOR_POSE_DATA_SIZE*(id-1), manipulatorPose);

  return manipulatorPose;
}

void RemoveManipulatorPresentPoseFromEEPROM(uint8_t id) {
  if (id >= 1 && id <= MANIPULATOR_POSE_ID_MAX_CNT)
    EEPROM.update(MANIPULATOR_POSE_DATA_SIZE*(id-1), 0);
}

ManipulatorPose RunManipulatorPoseWithPoseDataInEEPROM(Dynamixel2Arduino dxl, uint8_t id, int32_t operatingTimeMillis, float motor1Angle) {
  ManipulatorPose manipulatorPose = ReadManipulatorPresentPoseToEEPROM(id);

  if (manipulatorPose.isTherePoseData) {
    int32_t manipulatorMotor1Value = (motor1Angle == -360.0
                                      ? manipulatorPose.manipulatorMotor1Value
                                      : map(constrain(round(motor1Angle*100), -10000, 10000), -18000, 18000, 0, 4095));

    setManipulatorForwardMoveWithMotorValueForSyncWrite( dxl, manipulatorMotor1Value,
                                                         manipulatorPose.manipulatorMotor2Value,
                                                         manipulatorPose.manipulatorMotor3Value,
                                                         manipulatorPose.manipulatorMotor4Value,
                                                         operatingTimeMillis);
  }

  return manipulatorPose;
}
