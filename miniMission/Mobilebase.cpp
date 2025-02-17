#include "Arduino.h"
#include "Debug.h"
#include "Mobilebase.h"


// mobile velocity control mode sync write 용 객체 인스턴스화
sw_wheel_velocity_data_t sw_wheel_velocity_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_velocity_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_wheel_velocity_xels_sw[MOBILE_DXL_ID_CNT];


// mobile extended position control mode sync write 용 객체 인스턴스화
sw_wheel_position_data_t sw_wheel_position_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncWriteInst_t sw_mobile_position_infos; // syncwrite 정보 인스턴스 생성
// syncWrite할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncWrite_t info_wheel_position_xels_sw[MOBILE_DXL_ID_CNT];


// mobile extended position control mode sync read 용 객체 인스턴스화
uint8_t wheel_position_pkt_buf[wheel_position_pkt_buf_cap];
sr_wheel_position_data_t sr_wheel_position_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_mobile_position_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_wheel_position_xels_sr[MOBILE_DXL_ID_CNT];


// is moving sync read 용 객체 인스턴스화
uint8_t moving_pkt_buf[moving_pkt_buf_cap];
sr_wheel_moving_data_t sr_wheel_moving_data[MOBILE_DXL_ID_CNT]; // 모터 개수만큼 인스턴스 생성
DYNAMIXEL::InfoSyncReadInst_t sr_mobile_moving_infos; // syncread 정보 인스턴스 생성
// syncRead할 모터들 정보 인스턴스를 모터 개수만큼 생성
DYNAMIXEL::XELInfoSyncRead_t info_wheel_moving_xels_sr[MOBILE_DXL_ID_CNT];


//////////////  모바일베이스 함수 정의
bool InitMobilebase(Dynamixel2Arduino dxl) {
  // 모바일베이스 모터가 모두 있는지 확인
  if (!FindMobileBaseServos(dxl)) return false;
  
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);
  
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + MOBILEBASE_DEFAULT_DRIVE_MODE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + MOBILEBASE_DEFAULT_DRIVE_MODE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], MOBILEBASE_DEFAULT_OPERATING_MODE);

    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
  
  // mobilebase velocity control mode sync write 준비
  sw_mobile_velocity_infos.packet.p_buf = nullptr;
  sw_mobile_velocity_infos.packet.is_completed = false;
  sw_mobile_velocity_infos.addr = SW_WHEEL_VELOCITY_START_ADDR;
  sw_mobile_velocity_infos.addr_length = SW_WHEEL_VELOCITY_DATA_SIZE;
  sw_mobile_velocity_infos.p_xels = info_wheel_velocity_xels_sw;
  sw_mobile_velocity_infos.xel_count = 0;

  sw_wheel_velocity_data[0].goal_velocity = 0; // 모터에 write 할 데이터 초기화
  sw_wheel_velocity_data[1].goal_velocity = 0;
  sw_wheel_velocity_data[2].goal_velocity = 0;
  sw_wheel_velocity_data[3].goal_velocity = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_velocity_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_wheel_velocity_xels_sw[i].p_data = (uint8_t*)&sw_wheel_velocity_data[i];
    sw_mobile_velocity_infos.xel_count++;
  }
  sw_mobile_velocity_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // mobilebase velocity extended position control mode sync write 준비
  sw_mobile_position_infos.packet.p_buf = nullptr;
  sw_mobile_position_infos.packet.is_completed = false;
  sw_mobile_position_infos.addr = SW_WHEEL_POSITION_START_ADDR;
  sw_mobile_position_infos.addr_length = SW_WHEEL_POSITION_DATA_SIZE;
  sw_mobile_position_infos.p_xels = info_wheel_position_xels_sw;
  sw_mobile_position_infos.xel_count = 0;

  sw_wheel_position_data[0].profile_velocity = MOBILEBASE_DEFAULT_DRIVING_SPEED; // 모터에 write 할 데이터 초기화
  sw_wheel_position_data[0].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[0]);
  sw_wheel_position_data[1].profile_velocity = MOBILEBASE_DEFAULT_DRIVING_SPEED;
  sw_wheel_position_data[1].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[1]);
  sw_wheel_position_data[2].profile_velocity = MOBILEBASE_DEFAULT_DRIVING_SPEED;
  sw_wheel_position_data[2].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[2]);
  sw_wheel_position_data[3].profile_velocity = MOBILEBASE_DEFAULT_DRIVING_SPEED;
  sw_wheel_position_data[3].goal_position = dxl.readControlTableItem(PRESENT_POSITION, MOBILE_DXL_IDS[3]);

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sw[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sw[i].p_data = (uint8_t*)&sw_wheel_position_data[i];
    sw_mobile_position_infos.xel_count++;
  }
  sw_mobile_position_infos.is_info_changed = true; // sync write 정보가 변경됨을 설정
  
  // mobilebase extended position control mode sync read 준비
  sr_mobile_position_infos.packet.buf_capacity = wheel_position_pkt_buf_cap;
  sr_mobile_position_infos.packet.p_buf = wheel_position_pkt_buf;
  sr_mobile_position_infos.packet.is_completed = false;
  sr_mobile_position_infos.addr = SR_WHEEL_POSITION_START_ADDR;
  sr_mobile_position_infos.addr_length = SR_WHEEL_POSITION_DATA_SIZE;
  sr_mobile_position_infos.p_xels = info_wheel_position_xels_sr;
  sr_mobile_position_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_position_xels_sr[i].id = MOBILE_DXL_IDS[i];
    info_wheel_position_xels_sr[i].p_recv_buf = (uint8_t*)&sr_wheel_position_data[i];
    sr_mobile_position_infos.xel_count++;
  }
  sr_mobile_position_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  
  // mobilebase moving sync read 준비
  sr_mobile_moving_infos.packet.buf_capacity = moving_pkt_buf_cap;
  sr_mobile_moving_infos.packet.p_buf = moving_pkt_buf;
  sr_mobile_moving_infos.packet.is_completed = false;
  sr_mobile_moving_infos.addr = SR_WHEEL_MOVING_START_ADDR;
  sr_mobile_moving_infos.addr_length = SR_WHEEL_MOVING_DATA_SIZE;
  sr_mobile_moving_infos.p_xels = info_wheel_moving_xels_sr;
  sr_mobile_moving_infos.xel_count = 0;

  // 모터 정보 리스트에 모터 아이디 설정, 데이터 포인터 지정
  for(int i = 0; i < MOBILE_DXL_ID_CNT; i++) {
    info_wheel_moving_xels_sr[i].id = MOBILE_DXL_IDS[i];
    info_wheel_moving_xels_sr[i].p_recv_buf = (uint8_t*)&sr_wheel_moving_data[i];
    sr_mobile_moving_infos.xel_count++;
  }
  sr_mobile_moving_infos.is_info_changed = true; // sync read 정보가 변경됨을 설정
  
  return true;
}

void ChangeMobilebaseMode2VelocityControlMode(Dynamixel2Arduino dxl) {
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);// 각 모터에 설정
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + VELOCITY_BASED_PROFILE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + VELOCITY_BASED_PROFILE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], VELOCITY_CONTROL_MODE);
    
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
}

void ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(Dynamixel2Arduino dxl) {
  // 각 모터에 설정
  for (int i = 0 ; i < MOBILE_DXL_ID_CNT ; i++) {
    // 토크 끄기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_OFF);
    // 모드 설정
    // 모드 설정
    if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FL ||
        MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BL) { // 좌측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], NORMAL_MODE + TIME_BASED_PROFILE);
    } else if (MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_FR ||
               MOBILE_DXL_IDS[i] == MOBILE_DXL_ID_BR) { // 우측 바퀴
      dxl.writeControlTableItem(DRIVE_MODE, MOBILE_DXL_IDS[i], REVERSE_MODE + TIME_BASED_PROFILE);
    }
    dxl.writeControlTableItem(OPERATING_MODE, MOBILE_DXL_IDS[i], EXTENDED_POSITION_CONTROL_MODE);
    // 토크 켜기
    dxl.writeControlTableItem(TORQUE_ENABLE, MOBILE_DXL_IDS[i], TORQUE_ON);
  }
}

bool FindMobileBaseServos(Dynamixel2Arduino dxl) {
  uint8_t ids_pinged[10] = {0,};
  bool is_each_motor_found = true;
  if (uint8_t count_pinged = dxl.ping(DXL_BROADCAST_ID, ids_pinged, 
    sizeof(ids_pinged)/sizeof(ids_pinged[0]), 100)) {
    if (count_pinged >= MOBILE_DXL_ID_CNT) {
      uint8_t mobile_dxl_ids_idx = 0;
      uint8_t ids_pinged_idx = 0;
      while(1) {
        if (MOBILE_DXL_IDS[mobile_dxl_ids_idx]
            == ids_pinged[ids_pinged_idx++]) {
          mobile_dxl_ids_idx ++;

          if (mobile_dxl_ids_idx
              == sizeof(MOBILE_DXL_IDS)/sizeof(uint8_t)) {
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
#if defined(DEBUG) & 1
        DEBUG_SERIAL.print("Motor IDs does not match : ");
        DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      }
    } else {
#if defined(DEBUG) & 1
      DEBUG_SERIAL.print("Motor count does not match : ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
      is_each_motor_found = false;
    }
  } else{
#if defined(DEBUG) & 1
    DEBUG_SERIAL.print("Broadcast returned no items : ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    is_each_motor_found = false;
  }
  return is_each_motor_found;
}

bool CheckIfMobilebaseIsInPosition(Dynamixel2Arduino dxl) {
  uint8_t recv_cnt = 0;
  uint8_t isInPositionStatusSum = 0;
  while(1) {
    recv_cnt = dxl.syncRead(&sr_mobile_moving_infos);
    if(recv_cnt == MOBILE_DXL_ID_CNT){
#if defined(DEBUG) & 0
      DEBUG_SERIAL.print("[syncRead] Success, Received ID Count: ");
      DEBUG_SERIAL.println(recv_cnt);
#endif
      for(int i = 0 ; i < recv_cnt ; i++) {
        isInPositionStatusSum += (sr_wheel_moving_data[i].moving_status)&0x01;
#if defined(DEBUG) & 0
        DEBUG_SERIAL.print("  ID: ");
        DEBUG_SERIAL.print(sr_mobile_moving_infos.p_xels[i].id);
        DEBUG_SERIAL.print("\t is in position: ");
        DEBUG_SERIAL.println((sr_wheel_moving_data[i].moving_status)&0x01);
#endif
      }
      break;
    } else {
#if defined(DEBUG) & 1
      DEBUG_SERIAL.print("[syncRead] Fail, Lib error code: ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    }
  }

  return (isInPositionStatusSum == 4);
}

bool CheckIfMobilebaseIsMoving(Dynamixel2Arduino dxl) {
  uint8_t recv_cnt = 0;
  while(1) {
    recv_cnt = dxl.syncRead(&sr_mobile_moving_infos);
    if(recv_cnt == MOBILE_DXL_ID_CNT){
#if defined(DEBUG) & 0
      DEBUG_SERIAL.print("[syncRead] Success, Received ID Count: ");
      DEBUG_SERIAL.println(recv_cnt);
#endif
      for(int i = 0 ; i < recv_cnt ; i++) {
#if defined(DEBUG) & 0
        DEBUG_SERIAL.print("  ID: ");
        DEBUG_SERIAL.print(sr_mobile_moving_infos.p_xels[i].id);
        DEBUG_SERIAL.print("\t is in position: ");
        DEBUG_SERIAL.println(sr_wheel_moving_data[i].moving);
#endif
        if (sr_wheel_moving_data[i].moving == 1)
          return true;
      }
      break;
    } else {
#if defined(DEBUG) & 1
      DEBUG_SERIAL.print("[syncRead] Fail, Lib error code: ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    }
  }

  return false;
}

void DriveDistanceAndMmPerSecAndDirection(Dynamixel2Arduino dxl, float distance,
                                          uint8_t drivingDirection,
                                          int32_t mmPerSec) {
  int32_t motorValueForRotation = RADIANS_2_MOTOR_VALUE(distance/WHEEL_RADIUS_MM);

  switch(drivingDirection) {
    case DRIVE_DIRECTION_FORWARD:
      SetMobileRelativePositionForSyncWrite(dxl, motorValueForRotation, motorValueForRotation, motorValueForRotation, motorValueForRotation,
                                            (distance/mmPerSec)*S_TO_MILLIS_RATIO);
      break;
    case DRIVE_DIRECTION_BACKWARD:
      SetMobileRelativePositionForSyncWrite(dxl, -motorValueForRotation, -motorValueForRotation, -motorValueForRotation, -motorValueForRotation,
                                            (distance/mmPerSec)*S_TO_MILLIS_RATIO);
      break;
    case DRIVE_DIRECTION_LEFT:
      SetMobileRelativePositionForSyncWrite(dxl, -motorValueForRotation, motorValueForRotation, motorValueForRotation, -motorValueForRotation,
                                            (distance/mmPerSec)*S_TO_MILLIS_RATIO);
      break;
    case DRIVE_DIRECTION_RIGHT:
      SetMobileRelativePositionForSyncWrite(dxl, motorValueForRotation, -motorValueForRotation, -motorValueForRotation, motorValueForRotation,
                                            (distance/mmPerSec)*S_TO_MILLIS_RATIO);
      break;
  }
}

void DriveXYDistanceAndMmPerSec(Dynamixel2Arduino dxl, float xMm, float yMm, int32_t mmPerSec) {
  float distance = sqrt(xMm*xMm + yMm*yMm);
  
  float motor1ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm + xMm)/WHEEL_RADIUS_MM);
  float motor2ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm - xMm)/WHEEL_RADIUS_MM);
  float motor3ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm - xMm)/WHEEL_RADIUS_MM);
  float motor4ValueForRotation = RADIANS_2_MOTOR_VALUE((yMm + xMm)/WHEEL_RADIUS_MM);

  SetMobileRelativePositionForSyncWrite(dxl,
                                        motor1ValueForRotation,
                                        motor2ValueForRotation,
                                        motor3ValueForRotation,
                                        motor4ValueForRotation,
                                        (distance/mmPerSec)*S_TO_MILLIS_RATIO);
}

bool DriveForwardUntilDistanceWithTwoSensors(Dynamixel2Arduino dxl,
                                             int16_t leftSensorError, int16_t rightSensorError,
                                             int16_t toleranceValue, int32_t drivingSpeed) {
  if (abs(leftSensorError) < toleranceValue
      && abs(rightSensorError) < toleranceValue) { // 센서 값이 범위 내에 들 때
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0); // 정지
    return false;
  } else { // 센서 값이 범위 내에 들지 않을 때
    // 오차 = 실험값 - 이론값. PSD 센서를 기준으로 오차가 양수(물체와 가까울수록 값이 커짐)이면
    // 목표 거리보다 가까운 상태이므로 후진해야 함
    float leftSpeedRatio = (float)leftSensorError/(toleranceValue*5);
    if (!(abs(leftSpeedRatio) < 1.0)) leftSpeedRatio /= abs(leftSpeedRatio);  // 부호가 같은 1로 변경
    float rightSpeedRatio = (float)rightSensorError/(toleranceValue*5);
    if (!(abs(rightSpeedRatio) < 1.0)) rightSpeedRatio /= abs(rightSpeedRatio);
    
#if defined(DEBUG) & 0
        DEBUG_SERIAL.print("Mobilebase/GoForwardWithTwoSensors : leftSensorError : ");
        DEBUG_SERIAL.println(leftSensorError);
        DEBUG_SERIAL.print("Mobilebase/GoForwardWithTwoSensors : rightSensorError : ");
        DEBUG_SERIAL.println(rightSensorError);
        DEBUG_SERIAL.print("Mobilebase/GoForwardWithTwoSensors : leftSpeedRatio : ");
        DEBUG_SERIAL.println(leftSpeedRatio);
        DEBUG_SERIAL.print("Mobilebase/GoForwardWithTwoSensors : rightSpeedRatio : ");
        DEBUG_SERIAL.println(rightSpeedRatio);
#endif
    
    SetMobileGoalVelocityForSyncWrite(dxl, -round(drivingSpeed*leftSpeedRatio), -round(drivingSpeed*rightSpeedRatio),
                                      -round(drivingSpeed*leftSpeedRatio), -round(drivingSpeed*rightSpeedRatio));
    return true;
  }
}

bool LocateWithTwoSensors(Dynamixel2Arduino dxl,
                          int16_t sideSensorError, int16_t forwardSensorError,
                          int16_t sideSensorTolerance, int16_t forwardSensorTolerance,
                          float sideControlRatio, float forwardControlRatio,
                          uint8_t drivingDirectionLeftOrRight, int32_t drivingSpeed) {
  if (abs(sideSensorError) < sideSensorTolerance && abs(forwardSensorError) < forwardSensorTolerance) { // 센서 값이 범위 내에 들 때
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0); // 정지
    return false;
  } else { // 센서 값이 범위 내에 들지 않을 때
    // 오차 = 실험값 - 이론값. PSD 센서를 기준으로 오차가 양수(물체와 가까울수록 값이 커짐)이면
    // 목표 거리보다 가까운 상태이므로 후진해야 함
    float sideSpeedRatio = (float)sideSensorError*sideControlRatio;
    if (!(abs(sideSpeedRatio) < 1.0)) sideSpeedRatio /= abs(sideSpeedRatio);  // 부호가 같은 1로 변경
    float forwardSpeedRatio = (float)forwardSensorError*forwardControlRatio;
    if (!(abs(forwardSpeedRatio) < 1.0)) forwardSpeedRatio /= abs(forwardSpeedRatio);  // 부호가 같은 1로 변경
    
#if defined(DEBUG) & 0
    DEBUG_SERIAL.print("Mobilebase/LocateWithTwoSensors : sideSensorError : ");
    DEBUG_SERIAL.println(sideSensorError);
    DEBUG_SERIAL.print("Mobilebase/LocateWithTwoSensors : sideSpeedRatio : ");
    DEBUG_SERIAL.println(sideSpeedRatio);
    DEBUG_SERIAL.print("Mobilebase/LocateWithTwoSensors : forwardSensorError : ");
    DEBUG_SERIAL.println(forwardSensorError);
    DEBUG_SERIAL.print("Mobilebase/LocateWithTwoSensors : forwardSpeedRatio : ");
    DEBUG_SERIAL.println(forwardSpeedRatio);
#endif

    switch(drivingDirectionLeftOrRight) {
      case DRIVE_DIRECTION_LEFT:
        SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed*(-forwardSpeedRatio + sideSpeedRatio)), round(drivingSpeed*(-forwardSpeedRatio - sideSpeedRatio)),
                                          round(drivingSpeed*(-forwardSpeedRatio - sideSpeedRatio)), round(drivingSpeed*(-forwardSpeedRatio + sideSpeedRatio)));
        break;
      case DRIVE_DIRECTION_RIGHT:
        SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed*(-forwardSpeedRatio - sideSpeedRatio)), round(drivingSpeed*(-forwardSpeedRatio + sideSpeedRatio)),
                                          round(drivingSpeed*(-forwardSpeedRatio + sideSpeedRatio)), round(drivingSpeed*(-forwardSpeedRatio - sideSpeedRatio)));
        break;
    }
    
    return true;
  }
}

bool DriveUntilDistanceWithOneSensor(Dynamixel2Arduino dxl,
                                    int16_t sensorError, int16_t toleranceValue,
                                    uint8_t drivingDirection, int32_t drivingSpeed) {
  if (abs(sensorError) < toleranceValue) { // 센서 값이 범위 내에 들 때
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0); // 정지
    return false;
  } else { // 센서 값이 범위 내에 들지 않을 때
    // 오차 = 실험값 - 이론값. PSD 센서를 기준으로 오차가 양수(물체와 가까울수록 값이 커짐)이면
    // 목표 거리보다 가까운 상태이므로 후진해야 함
    float speedRatio = (float)sensorError/(toleranceValue*5);
    if (!(abs(speedRatio) < 1.0)) speedRatio /= abs(speedRatio);  // 부호가 같은 1로 변경
    
#if defined(DEBUG) & 0
    DEBUG_SERIAL.print("Mobilebase/DriveUntilDistanceWithOneSensor : sensorError : ");
    DEBUG_SERIAL.println(sensorError);
    DEBUG_SERIAL.print("Mobilebase/DriveUntilDistanceWithOneSensor : speedRatio : ");
    DEBUG_SERIAL.println(speedRatio);
#endif

    switch(drivingDirection) {
      case DRIVE_DIRECTION_FORWARD:
        SetMobileGoalVelocityForSyncWrite(dxl, -round(drivingSpeed*speedRatio), -round(drivingSpeed*speedRatio),
                                          -round(drivingSpeed*speedRatio), -round(drivingSpeed*speedRatio));
        break;
      case DRIVE_DIRECTION_BACKWARD:
        SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed*speedRatio), round(drivingSpeed*speedRatio),
                                          round(drivingSpeed*speedRatio), round(drivingSpeed*speedRatio));
        break;
      case DRIVE_DIRECTION_LEFT:
        SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed*speedRatio), -round(drivingSpeed*speedRatio),
                                          -round(drivingSpeed*speedRatio), round(drivingSpeed*speedRatio));
        break;
      case DRIVE_DIRECTION_RIGHT:
        SetMobileGoalVelocityForSyncWrite(dxl, -round(drivingSpeed*speedRatio), round(drivingSpeed*speedRatio),
                                          round(drivingSpeed*speedRatio), -round(drivingSpeed*speedRatio));
        break;
    }
    
    return true;
  }
}

/*
 * x위치, y위치, 회전 에러 값을 사용하여 정해진 방향으로 주행하는 함수
 * params :
 *    xPosSensorError : x 거리 센서 오차
 *    yPosSensorError : y 거리 센서 오차
 *    rotationSensorError : 회전 센서 오차
 *    mainPosToleranceValue : mainDrivingDirection 방향의 거리 허용 오차
 *    subPosToleranceValue : subDrivingDirection 방향의 거리 허용 오차
 *    rotationToleranceValue : 회전 허용 오차
 *    mainDrivingDirection : 주 주행 방향. 오차가 음수일 때 회전할 방향을 의미
 *    subDrivingDirection : 보조 주행 방향. 오차가 음수일 때 회전할 방향을 의미
 *    rotationDirection : 회전 방향. 오차가 음수일 때 회전할 방향을 의미
 *    drivingSpeed : 주행 속도
 *    rotatingSpeed : 회전 속도
 * return :
 *    true : 제어중
 *    false : 제어 완료
 */
bool DriveWithPositionAndRotationErrors(Dynamixel2Arduino dxl, int16_t xPosSensorError, int16_t yPosSensorError, int16_t rotationSensorError,
                                        int16_t mainPosToleranceValue, int16_t subPosToleranceValue, int16_t rotationToleranceValue,
                                        float mainControlRatio, float subControlRatio, float roatingControlRatio,
                                        uint8_t mainDrivingDirection, uint8_t subDrivingDirection,uint8_t rotatingDirection,
                                        int32_t drivingSpeed, int32_t rotatingSpeed) {
  // todo 잘못된 direction 입력에 대한 처리 필요
  int16_t mainDistanceSensorError = ((mainDrivingDirection == DRIVE_DIRECTION_LEFT) || (mainDrivingDirection == DRIVE_DIRECTION_RIGHT)
                                      ? xPosSensorError : yPosSensorError);
  int16_t subDistanceSensorError = ((subDrivingDirection == DRIVE_DIRECTION_LEFT) || (subDrivingDirection == DRIVE_DIRECTION_RIGHT)
                                      ? xPosSensorError : yPosSensorError );

  if (abs(mainDistanceSensorError) < mainPosToleranceValue
      && abs(subDistanceSensorError) < subPosToleranceValue
      && abs(rotationSensorError) < rotationToleranceValue) { // 센서 값이 범위 내에 들 때
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0); // 정지
    return false;
  } else { // 센서 값이 범위 내에 들지 않을 때
    // 오차 = 실험값 - 이론값. 사인값을 기준으로 오차가 음수이면
    // 목표 각도보다 시계 방향으로 더 회전한 상태이므로 반시계 방향으로 회전해야 함
    float mainDrivingSpeedRatio = (float)mainDistanceSensorError*mainControlRatio;
    if (abs(mainDrivingSpeedRatio) > 1.0) mainDrivingSpeedRatio /= abs(mainDrivingSpeedRatio);  // 부호가 같은 1로 변경
    float subDrivingControl = (float)subDistanceSensorError*subControlRatio;
    if (abs(subDrivingControl) > 1.0) subDrivingControl /= abs(subDrivingControl);  // 부호가 같은 1로 변경
    float rotatingControl = (float)rotationSensorError*roatingControlRatio/2.0;
    if (abs(rotatingControl) > 1.0) rotatingControl /= abs(rotatingControl);  // 부호가 같은 1로 변경
    
#if defined(DEBUG) & 0
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : mainDistanceSensorError : ");
    DEBUG_SERIAL.println(mainDistanceSensorError);
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : subDistanceSensorError : ");
    DEBUG_SERIAL.println(subDistanceSensorError);
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : rotationSensorError : ");
    DEBUG_SERIAL.println(rotationSensorError);
    
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : mainDrivingSpeedRatio : ");
    DEBUG_SERIAL.println(mainDrivingSpeedRatio);
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : subDrivingControl : ");
    DEBUG_SERIAL.println(subDrivingControl);
    DEBUG_SERIAL.print("Mobilebase/DriveWithPositionAndRotationErrors : rotatingControl : ");
    DEBUG_SERIAL.println(rotatingControl);
#endif

    int32_t fl_velocity = 0;
    int32_t fr_velocity = 0;
    int32_t bl_velocity = 0;
    int32_t br_velocity = 0;
    
    switch(mainDrivingDirection) {
      case DRIVE_DIRECTION_FORWARD:
        fl_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        fr_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        bl_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        br_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        break;
      case DRIVE_DIRECTION_BACKWARD:
        fl_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        fr_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        bl_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        br_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        break;
      case DRIVE_DIRECTION_LEFT:
        fl_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        fr_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        bl_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        br_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        break;
      case DRIVE_DIRECTION_RIGHT:
        fl_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        fr_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        bl_velocity += round(drivingSpeed*mainDrivingSpeedRatio);
        br_velocity += -round(drivingSpeed*mainDrivingSpeedRatio);
        break;
    }
    
    switch(subDrivingDirection) {
      case DRIVE_DIRECTION_FORWARD:
        fl_velocity += -round(drivingSpeed*subDrivingControl);
        fr_velocity += -round(drivingSpeed*subDrivingControl);
        bl_velocity += -round(drivingSpeed*subDrivingControl);
        br_velocity += -round(drivingSpeed*subDrivingControl);
        break;
      case DRIVE_DIRECTION_BACKWARD:
        fl_velocity += round(drivingSpeed*subDrivingControl);
        fr_velocity += round(drivingSpeed*subDrivingControl);
        bl_velocity += round(drivingSpeed*subDrivingControl);
        br_velocity += round(drivingSpeed*subDrivingControl);
        break;
      case DRIVE_DIRECTION_LEFT:
        fl_velocity += round(drivingSpeed*subDrivingControl);
        fr_velocity += -round(drivingSpeed*subDrivingControl);
        bl_velocity += -round(drivingSpeed*subDrivingControl);
        br_velocity += round(drivingSpeed*subDrivingControl);
        break;
      case DRIVE_DIRECTION_RIGHT:
        fl_velocity += -round(drivingSpeed*subDrivingControl);
        fr_velocity += round(drivingSpeed*subDrivingControl);
        bl_velocity += round(drivingSpeed*subDrivingControl);
        br_velocity += -round(drivingSpeed*subDrivingControl);
        break;
    }
    
    switch(rotatingDirection) {
      case ROTATE_CCW:
        fl_velocity += round(rotatingSpeed*rotatingControl);
        fr_velocity += -round(rotatingSpeed*rotatingControl);
        bl_velocity += round(rotatingSpeed*rotatingControl);
        br_velocity += -round(rotatingSpeed*rotatingControl);
        break;
      case ROTATE_CW:
        fl_velocity += -round(rotatingSpeed*rotatingControl);
        fr_velocity += round(rotatingSpeed*rotatingControl);
        bl_velocity += -round(rotatingSpeed*rotatingControl);
        br_velocity += round(rotatingSpeed*rotatingControl);
        break;
    }
    
    SetMobileGoalVelocityForSyncWrite(dxl, fl_velocity, fr_velocity,
                                      bl_velocity, br_velocity);
    
    return true;
  }
}

bool DriveUntilNoObstacleWithOneSensor(Dynamixel2Arduino dxl, int16_t sensorValue, int16_t thresholdValue,
                                       uint8_t drivingDirection, int32_t drivingSpeed) {
  if (sensorValue > thresholdValue) {
    if (!CheckIfMobilebaseIsMoving(dxl)) {
      switch(drivingDirection) {
        case DRIVE_DIRECTION_FORWARD:
          SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed), round(drivingSpeed),
                                            round(drivingSpeed), round(drivingSpeed));
          break;
        case DRIVE_DIRECTION_BACKWARD:
          SetMobileGoalVelocityForSyncWrite(dxl, -round(drivingSpeed), -round(drivingSpeed),
                                            -round(drivingSpeed), -round(drivingSpeed));
          break;
        case DRIVE_DIRECTION_LEFT:
          SetMobileGoalVelocityForSyncWrite(dxl, -round(drivingSpeed), round(drivingSpeed),
                                            round(drivingSpeed), -round(drivingSpeed));
          break;
        case DRIVE_DIRECTION_RIGHT:
          SetMobileGoalVelocityForSyncWrite(dxl, round(drivingSpeed), -round(drivingSpeed),
                                            -round(drivingSpeed), round(drivingSpeed));
          break;
      }
    }
      
    return true;
  } else {
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
    return false;
  }
}

void SetMobileRelativePositionForSyncWrite(Dynamixel2Arduino dxl,
                                           int32_t fl_relative_position, int32_t fr_relative_position,
                                           int32_t bl_relative_position, int32_t br_relative_position,
                                           int32_t drivingTime) {
  uint8_t recv_cnt = 0;
  while(1) {
    recv_cnt = dxl.syncRead(&sr_mobile_position_infos);
    if(recv_cnt == MOBILE_DXL_ID_CNT){
#if defined(DEBUG) & 0
      DEBUG_SERIAL.print("[syncRead] Success, Received ID Count: ");
      DEBUG_SERIAL.println(recv_cnt);
#endif
      for(int i = 0 ; i < recv_cnt ; i++) {
#if defined(DEBUG) & 0
        DEBUG_SERIAL.print("  ID: ");
        DEBUG_SERIAL.print(sr_mobile_position_infos.p_xels[i].id);
        DEBUG_SERIAL.print("\t Present Position: ");
        DEBUG_SERIAL.println(sr_wheel_position_data[i].present_position);
#endif
      }
      break;
    } else {
#if defined(DEBUG) & 1
      DEBUG_SERIAL.print("[syncRead] Fail, Lib error code: ");
      DEBUG_SERIAL.println(dxl.getLastLibErrCode());
#endif
    }
  }

  if (abs(sr_wheel_position_data[0].present_position + fl_relative_position) > 1048575
      || abs(sr_wheel_position_data[1].present_position + fr_relative_position) > 1048575
      || abs(sr_wheel_position_data[2].present_position + bl_relative_position) > 1048575
      || abs(sr_wheel_position_data[3].present_position + br_relative_position) > 1048575) {
#if defined(DEBUG) & 1
    DEBUG_SERIAL.println("SetMobileRelativePositionForSyncWrite failure. please reboot motor");
#endif 
  } else {
    sw_wheel_position_data[0].profile_velocity = drivingTime;
    sw_wheel_position_data[0].goal_position = sr_wheel_position_data[0].present_position + fl_relative_position;
    sw_wheel_position_data[1].profile_velocity = drivingTime;
    sw_wheel_position_data[1].goal_position = sr_wheel_position_data[1].present_position + fr_relative_position;
    sw_wheel_position_data[2].profile_velocity = drivingTime;
    sw_wheel_position_data[2].goal_position = sr_wheel_position_data[2].present_position + bl_relative_position;
    sw_wheel_position_data[3].profile_velocity = drivingTime;
    sw_wheel_position_data[3].goal_position = sr_wheel_position_data[3].present_position + br_relative_position;
  }
  
  sw_mobile_position_infos.is_info_changed = true;
  
  while(!dxl.syncWrite(&sw_mobile_position_infos)) {}
}

void SetMobileGoalVelocityForSyncWrite(Dynamixel2Arduino dxl,
                                       int32_t fl_goal_velocity,
                                       int32_t fr_goal_velocity,
                                       int32_t bl_goal_velocity,
                                       int32_t br_goal_velocity) {
  sw_wheel_velocity_data[0].goal_velocity = fl_goal_velocity;
  sw_wheel_velocity_data[1].goal_velocity = fr_goal_velocity;
  sw_wheel_velocity_data[2].goal_velocity = bl_goal_velocity;
  sw_wheel_velocity_data[3].goal_velocity = br_goal_velocity;
  sw_mobile_velocity_infos.is_info_changed = true;
  delay(1);
  while(!dxl.syncWrite(&sw_mobile_velocity_infos)) {}
}
