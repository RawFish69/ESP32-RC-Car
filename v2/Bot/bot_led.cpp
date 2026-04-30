#include "bot_led.h"

#include "bot_state.h"

namespace bot {

void set_led_color(uint8_t r, uint8_t g, uint8_t b)
{
  status_led.setPixelColor(0, status_led.Color(r, g, b));
  status_led.show();
}

void led_error()
{
  led_base_color = status_led.Color(200, 0, 0);
  set_led_color(200, 0, 0);
}

void led_waiting()
{
  led_base_color = status_led.Color(200, 100, 0);
  set_led_color(200, 100, 0);
}

void trigger_activity()
{
  activity_flag = true;
}

void update_led()
{
  if (activity_flag)
  {
    activity_flag = false;
    if (led_base_color != status_led.Color(200, 0, 0))
    {
      led_base_color = status_led.Color(0, 200, 0);
    }
    led_flash_end = millis() + 300;
    set_led_color(0, 0, 200);
  }
  else if (millis() > led_flash_end)
  {
    const uint8_t r = (led_base_color >> 16) & 0xFF;
    const uint8_t g = (led_base_color >> 8) & 0xFF;
    const uint8_t b = led_base_color & 0xFF;
    static uint32_t last_shown = 0xFFFFFFFF;
    if (led_base_color != last_shown)
    {
      set_led_color(r, g, b);
      last_shown = led_base_color;
    }
  }
}

void setup_led()
{
  status_led.begin();
  status_led.setBrightness(40);
  led_waiting();
}

} // namespace bot
