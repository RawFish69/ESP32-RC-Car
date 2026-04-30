#include "bot_imu.h"

#ifdef BOT_HAS_IMU

// Include your IMU library here, e.g.:
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>

namespace bot {

void setup_imu()
{
  // TODO: initialize IMU over I2C using kImuSdaPin / kImuSclPin from bot_config.h
}

float get_heading_deg()
{
  // TODO: return fused yaw estimate
  return 0.0f;
}

float get_pitch_deg()
{
  // TODO: return pitch from accelerometer
  return 0.0f;
}

float get_roll_deg()
{
  // TODO: return roll from accelerometer
  return 0.0f;
}

} // namespace bot

#endif // BOT_HAS_IMU
