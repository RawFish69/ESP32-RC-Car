#pragma once

// Optional: IMU support (e.g. MPU-6050, ICM-42688-P)
//
// Not required for the basic build — the bot runs fine with just TT motors and LiDAR.
// Enable by defining BOT_HAS_IMU in bot_config.h and filling in the implementation.
// Useful for heading estimation and tilt compensation in more advanced navigation.

#ifdef BOT_HAS_IMU

#include <Arduino.h>

namespace bot {

void  setup_imu();
float get_heading_deg();  // yaw estimate from gyro/magnetometer fusion
float get_pitch_deg();    // pitch from accelerometer
float get_roll_deg();     // roll from accelerometer

} // namespace bot

#endif // BOT_HAS_IMU
