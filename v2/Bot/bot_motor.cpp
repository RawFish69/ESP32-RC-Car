#include "bot_motor.h"

#include "bot_config.h"
#include "bot_led.h"
#include "bot_state.h"

namespace bot {

void enable_motor_driver(bool enabled)
{
  if (kMotorStandbyPin >= 0)
  {
    digitalWrite(kMotorStandbyPin, enabled ? HIGH : LOW);
  }
}

void set_motor(int pwm_pin, int in1_pin, int in2_pin, int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed == 0)
  {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    ledcWrite(pwm_pin, 0);
    return;
  }

  enable_motor_driver(true);
  if (speed > 0)
  {
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  }
  else
  {
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
    speed = -speed;
  }

  const uint32_t duty = kLedcMax * static_cast<uint32_t>(speed) / 255U;
  ledcWrite(pwm_pin, duty);
}

void drive(int left_speed, int right_speed)
{
  set_motor(kLeftPwmPin,
            kLeftMotorIn1Pin,
            kLeftMotorIn2Pin,
            left_speed * kLeftMotorPolarity);
  set_motor(kRightPwmPin,
            kRightMotorIn1Pin,
            kRightMotorIn2Pin,
            right_speed * kRightMotorPolarity);

  if (left_speed == 0 && right_speed == 0)
  {
    enable_motor_driver(false);
  }
}

void forward()
{
  drive(kDriveSpeed, kDriveSpeed);
}

void backward()
{
  drive(-kDriveSpeed, -kDriveSpeed);
}

void reverse_short()
{
  drive(-kReverseSpeed, -kReverseSpeed);
}

void turn_left()
{
  drive(kTurnSlow, kTurnFast);
}

void turn_right()
{
  drive(kTurnFast, kTurnSlow);
}

void spin_left()
{
  drive(-kSpinSpeed, kSpinSpeed);
}

void spin_right()
{
  drive(kSpinSpeed, -kSpinSpeed);
}

void stop_drive()
{
  drive(0, 0);
}

void control_motor(char cmd)
{
  switch (cmd)
  {
    case 'w':
      forward();
      break;
    case 's':
      backward();
      break;
    case 'a':
      turn_left();
      break;
    case 'd':
      turn_right();
      break;
    case 'q':
      spin_left();
      break;
    case 'e':
      spin_right();
      break;
    case 'x':
      stop_drive();
      break;
    default:
      break;
  }
}

void apply_motor_cmd(char motor, char dir, uint8_t pwm)
{
  const int speed = (dir == 'f') ? static_cast<int>(pwm)
                                 : (dir == 'b') ? -static_cast<int>(pwm) : 0;
  if (motor == 'L')
  {
    set_motor(kLeftPwmPin, kLeftMotorIn1Pin, kLeftMotorIn2Pin, speed * kLeftMotorPolarity);
  }
  else
  {
    set_motor(kRightPwmPin, kRightMotorIn1Pin, kRightMotorIn2Pin, speed * kRightMotorPolarity);
  }

  trigger_activity();
  Serial.printf("Motor %c: %c PWM %u\n", motor, dir, static_cast<unsigned>(pwm));
}

void setup_motor()
{
  ledcAttach(kLeftPwmPin, kLedcFreqHz, kLedcBits);
  ledcAttach(kRightPwmPin, kLedcFreqHz, kLedcBits);
  pinMode(kLeftMotorIn1Pin, OUTPUT);
  pinMode(kLeftMotorIn2Pin, OUTPUT);
  pinMode(kRightMotorIn1Pin, OUTPUT);
  pinMode(kRightMotorIn2Pin, OUTPUT);
  if (kMotorStandbyPin >= 0)
  {
    pinMode(kMotorStandbyPin, OUTPUT);
    enable_motor_driver(true);
  }
  stop_drive();

  Serial.println("TB6612FNG motors ready.");
  Serial.printf("  Left: PWMA=%d AIN1=%d AIN2=%d\n",
                kLeftPwmPin,
                kLeftMotorIn1Pin,
                kLeftMotorIn2Pin);
  Serial.printf("  Right: PWMB=%d BIN1=%d BIN2=%d\n",
                kRightPwmPin,
                kRightMotorIn1Pin,
                kRightMotorIn2Pin);
  if (kMotorStandbyPin >= 0)
  {
    Serial.printf("  STBY=%d\n", kMotorStandbyPin);
  }
  else
  {
    Serial.println("  STBY tied high on driver board");
  }
  Serial.println("  1=manual  4=basic wander  x=stop");
}

} // namespace bot
