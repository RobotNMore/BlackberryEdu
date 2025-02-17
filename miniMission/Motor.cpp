#include "Arduino.h"
#include <Dynamixel2Arduino.h>
#include "Motor.h"

void InitMotorCommunication(Dynamixel2Arduino dxl) {
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
}
