#include "bot_comms.h"

#include <WiFi.h>
#include <ctype.h>
#include <string.h>

#include "bot_behaviors.h"
#include "bot_led.h"
#include "bot_lidar.h"
#include "bot_motor.h"
#include "bot_state.h"

namespace bot {

void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len)
{
  if (len < 1)
  {
    return;
  }

  if (info != nullptr && info->src_addr != nullptr)
  {
    if (!controller_peer_known ||
        memcmp(controller_peer_addr, info->src_addr, sizeof(controller_peer_addr)) != 0)
    {
      if (controller_peer_known && esp_now_is_peer_exist(controller_peer_addr))
      {
        esp_now_del_peer(controller_peer_addr);
      }

      esp_now_peer_info_t controller_peer = {};
      memcpy(controller_peer.peer_addr, info->src_addr, sizeof(controller_peer.peer_addr));
      controller_peer.channel = 0;
      controller_peer.encrypt = false;

      if (esp_now_add_peer(&controller_peer) == ESP_OK)
      {
        memcpy(controller_peer_addr, info->src_addr, sizeof(controller_peer_addr));
        controller_peer_known = true;
      }
    }
  }

  if (len == 3 && (data[0] == 'L' || data[0] == 'R'))
  {
    apply_motor_cmd(static_cast<char>(data[0]), static_cast<char>(data[1]), data[2]);
    last_cmd_time = millis();
    return;
  }

  if (!isascii(data[0]))
  {
    return;
  }

  const char cmd = static_cast<char>(data[0]);
  if (cmd == 'v')
  {
    return;
  }
  trigger_activity();

  if (cmd == '1' || cmd == '4' || cmd == 'x')
  {
    activate_mode(cmd);
  }
  else if (mode == '1')
  {
    direction = cmd;
    last_cmd_time = millis();
    control_motor(cmd);
  }
}

void setup_espnow()
{
  WiFi.mode(WIFI_STA);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW init failed, restarting");
    led_error();
    delay(2000);
    ESP.restart();
  }

  esp_now_register_recv_cb(on_data_recv);
}

void handle_usb_serial()
{
  static String serial_buf;

  while (Serial.available())
  {
    const char k = static_cast<char>(Serial.read());
    if (k == '\r')
    {
      continue;
    }

    if (k == '\n')
    {
      if (serial_buf == "lp")
      {
        lidar_reader.print_packet_summary(Serial);
      }
      else if (serial_buf == "ls")
      {
        print_lidar_status();
      }
      else if (serial_buf.length() >= 2)
      {
        apply_motor_cmd(serial_buf[0],
                        serial_buf[1],
                        serial_buf.length() > 2
                            ? static_cast<uint8_t>(
                                  constrain(serial_buf.substring(2).toInt(), 0, 255))
                            : 0);
      }
      serial_buf = "";
      continue;
    }

    if (serial_buf.length() > 0)
    {
      serial_buf += k;
      continue;
    }

    if (k == '1' || k == '4' || k == 'x')
    {
      activate_mode(k);
    }
    else if (k == 'L' || k == 'R' || k == 'l')
    {
      serial_buf = k;
    }
    else if (mode == '1')
    {
      direction = k;
      last_cmd_time = millis();
      control_motor(k);
    }
  }
}

} // namespace bot
