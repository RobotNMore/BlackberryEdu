/*
 * IMU 라이브러리 다운로드 링크
 * http://librarymanager/All#SparkFun_ICM_20948_IMU
 * 
 * 다운로드 후 DMP를 활성화 하기 위해 아두이노 라이브러리가 있는 경로에서
 * [SparkFun_9DoF_IMU_Breakout_-_ICM_20948_-_Arduino_Library
 *  -> src -> util]
 * 경로의 파일 ICM_20948_C.h 를 열고 29번째 줄에 있는
 * ICM_20948_USE_DMP를 주석 해제
 */

//////////////  9Axis IMU
#include "ICM_20948.h"    // SparkFun IMU 라이브러리 포함
#define WIRE_PORT Wire    // 통신에 사용할 객체 정의
#define AD0_VAL 0         // IMU 모듈 주소 정의. 블랙베리의 IMU 주소값 0x68을 의미
ICM_20948_I2C imu;        // imu 객체 생성
//#define QUAT_ANIMATION  // 센서 자세를 Node.js를 사용한 애니메이션으로 보려면
                          // 주석을 해제: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

void setup()
{
  Serial.begin(115200); // 시리얼 모니터 통신 설정
#ifndef QUAT_ANIMATION
  Serial.println(F("ICM-20948 Example"));
#endif

  delay(100);

#ifndef QUAT_ANIMATION
  while (Serial.available()) // 시리얼 모니터 수신 버퍼 비우기
    Serial.read();

  Serial.println(F("Press any key to continue..."));
  
  while (!Serial.available()); // 시리얼 모니터에 아무 값이나 입력되길 기다림
#endif

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

#ifndef QUAT_ANIMATION
//  imu.enableDebugging(); // 시리얼 모니터에 imu 디버깅용 메시지를 띄우려면
#endif                     // 주석을 해제

  bool initialized = false;
  while (!initialized)
  {
    // ICM-20948를 초기화
    // DMP가 사용 가능한 경우, .begin함수는 최소한의 시작을 수행함
    // 샘플 모드 등은 수동으로 설정해야 함
    imu.begin(WIRE_PORT, AD0_VAL);

#ifndef QUAT_ANIMATION
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(imu.statusString());
#endif
    if (imu.status != ICM_20948_Stat_Ok)
    {
#ifndef QUAT_ANIMATION
      Serial.println(F("Trying again..."));
#endif
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

#ifndef QUAT_ANIMATION
  Serial.println(F("Device connected!"));
#endif

  bool success = true; // DMP 설정이 완료되었는지를 나타내는 플래그

  success &= (imu.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION)
               == ICM_20948_Stat_Ok);
  success &= (imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 0)
               == ICM_20948_Stat_Ok); // 최대로 설정
  success &= (imu.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (imu.enableDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetDMP() == ICM_20948_Stat_Ok);
  success &= (imu.resetFIFO() == ICM_20948_Stat_Ok);

  // 설정에 성공했는지 확인
  if (success)
  {
#ifndef QUAT_ANIMATION
    Serial.println(F("DMP enabled!"));
#endif
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29"\
                      "(#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // 더 이상 아무것도 하지 않음
  }
}

void loop()
{
  icm_20948_DMP_data_t data;
  imu.readDMPdataFromFIFO(&data);

  // 유효한 데이터가 사용 가능한지 확인
  if ((imu.status == ICM_20948_Stat_Ok)
       || (imu.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    // 업데이트된 data가 Quat9 데이터인지 data header에 비트마스크를 사용해 확인
    if ((data.header & DMP_header_bitmap_Quat9) > 0)
    {
      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      // 쿼터니언을 오일러 각도로 변환 (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2
      double q2sqr = q2 * q2;

      // pitch (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double pitch = atan2(t0, t1) * 180.0 / PI;

      // roll (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double roll = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;
      
#ifndef QUAT_ANIMATION
      // Roll Pitch Yaw 각도를 시리얼 모니터에 출력
      Serial.print(F("Roll:"));
      Serial.print(roll, 1);
      Serial.print(F(" Pitch:"));
      Serial.print(pitch, 1);
      Serial.print(F(" Yaw:"));
      Serial.println(yaw, 1);
#else
      // 센서 자세를 Node.js를 사용한 애니메이션으로 보기 위한
      // 포맷에 따라 쿼터니언 데이터를 출력
      // https://github.com/ZaneL/quaternion_sensor_3d_nodejs
      Serial.print(F("{\"quat_w\":"));
      Serial.print(q0, 3);
      Serial.print(F(", \"quat_x\":"));
      Serial.print(q1, 3);
      Serial.print(F(", \"quat_y\":"));
      Serial.print(q2, 3);
      Serial.print(F(", \"quat_z\":"));
      Serial.print(q3, 3);
      Serial.println(F("}"));
#endif
    }
  }

  // 사용 가능한 데이터가 더 남아있다면 딜레이 없이 바로 다음으로 넘어감
  if (imu.status != ICM_20948_Stat_FIFOMoreDataAvail)
  {
    delay(10);
  }
}
