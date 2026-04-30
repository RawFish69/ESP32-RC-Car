#include "bot_encoder.h"

#ifdef BOT_HAS_ENCODERS

#include <math.h>
#include "bot_config.h"

namespace bot {
namespace {

volatile long left_ticks  = 0;
volatile long right_ticks = 0;

void IRAM_ATTR left_isr()  { left_ticks++;  }
void IRAM_ATTR right_isr() { right_ticks++; }

float left_integral  = 0.0f;
float right_integral = 0.0f;
int   last_left_err  = 0;
int   last_right_err = 0;

} // namespace

void setup_encoders()
{
  pinMode(kLeftEncoderPin,  INPUT_PULLUP);
  pinMode(kRightEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderPin),  left_isr,  RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderPin), right_isr, RISING);
}

void reset_odometry()
{
  left_ticks  = 0;
  right_ticks = 0;
}

float get_left_speed_mps()
{
  // TODO: compute speed from tick rate over a rolling time window
  return 0.0f;
}

float get_right_speed_mps()
{
  // TODO: compute speed from tick rate over a rolling time window
  return 0.0f;
}

float get_distance_m()
{
  const float circumference = kWheelDiameterM * (float)M_PI;
  const long  avg_ticks     = (left_ticks + right_ticks) / 2;
  return (avg_ticks / (float)kEncoderPpr) * circumference;
}

void update_pid(int target_left_pwm, int target_right_pwm)
{
  // TODO: read current speed, compute PID error, and call drive() with corrected values
  (void)target_left_pwm;
  (void)target_right_pwm;
}

} // namespace bot

#endif // BOT_HAS_ENCODERS
