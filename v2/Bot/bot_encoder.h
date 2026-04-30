#pragma once

// Optional: Motor encoder support
//
// Not required for the basic build — the bot runs fine without encoders.
// Enable by defining BOT_HAS_ENCODERS in bot_config.h and filling in the implementation.
// Encoders enable closed-loop PID speed control and wheel odometry, both of which
// are needed for accurate SLAM.

#ifdef BOT_HAS_ENCODERS

#include <Arduino.h>

namespace bot {

void  setup_encoders();
void  reset_odometry();
float get_left_speed_mps();               // left wheel speed in m/s
float get_right_speed_mps();              // right wheel speed in m/s
float get_distance_m();                   // average forward distance traveled
void  update_pid(int target_left_pwm,
                 int target_right_pwm);   // closed-loop speed correction

} // namespace bot

#endif // BOT_HAS_ENCODERS
