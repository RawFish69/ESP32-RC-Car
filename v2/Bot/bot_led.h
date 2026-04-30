#pragma once

#include <Arduino.h>

namespace bot {

void set_led_color(uint8_t r, uint8_t g, uint8_t b);
void led_error();
void led_waiting();
void trigger_activity();
void update_led();
void setup_led();

} // namespace bot
