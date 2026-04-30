#pragma once

#include <Arduino.h>

namespace bot {

void enable_motor_driver(bool enabled);
void set_motor(int pwm_pin, int in1_pin, int in2_pin, int speed);
void drive(int left_speed, int right_speed);
void forward();
void backward();
void reverse_short();
void turn_left();
void turn_right();
void spin_left();
void spin_right();
void stop_drive();
void control_motor(char cmd);
void apply_motor_cmd(char motor, char dir, uint8_t pwm);
void setup_motor();

} // namespace bot
