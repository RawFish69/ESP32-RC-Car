#include <Arduino.h>

#include "bot_behaviors.h"
#include "bot_comms.h"
#include "bot_config.h"
#include "bot_led.h"
#include "bot_lidar.h"
#include "bot_motor.h"
#include "bot_state.h"

void setup()
{
  Serial.begin(115200);
  bot::setup_led();
  randomSeed(esp_random());
  bot::setup_motor();
  bot::setup_lidar();
  bot::setup_espnow();

  Serial.println("USB commands:");
  Serial.println("  1 = manual drive (teleop)");
  Serial.println("  4 = basic wander mode");
  Serial.println("  x = stop");
  Serial.println("  Lf200 / Rb150 / Ls = direct motor control (uppercase L/R)");
  Serial.println("  lp = print last LD06 packet (lowercase l)");
  Serial.println("  ls = print latest LD06 sector summary (lowercase l)");
}

void loop()
{
  bot::handle_usb_serial();
  bot::update_lidar();

  if (bot::g_dodging && millis() >= bot::g_dodge_end_ms)
  {
    bot::g_dodging = false;
  }

  if (bot::g_unsticking && millis() >= bot::g_unstuck_end_ms)
  {
    bot::g_unsticking = false;
  }

  if (!bot::g_dodging && !bot::g_unsticking && bot::maybe_start_unstuck())
  {
    bot::maybe_report_lidar();
    bot::update_led();
    delay(10);
    return;
  }

  if (!bot::g_dodging && !bot::g_unsticking)
  {
    switch (bot::mode)
    {
      // Basic wander mode: forward drive with LiDAR obstacle avoidance.
      // Modes 2, 3, and 5 are not included in this release.
      case '4': bot::basic_wander(); break;

      case '1':
        if (bot::direction != 'x' && millis() - bot::last_cmd_time > bot::kTeleopTimeoutMs)
        {
          bot::direction = 'x';
          bot::stop_drive();
          Serial.println("Teleop timeout -> stopped");
        }
        break;

      case 'x':
      default:
        break;
    }
  }

  bot::maybe_report_lidar();
  bot::update_led();
  delay(10);
}
