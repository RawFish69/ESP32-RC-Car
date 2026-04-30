#pragma once

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "LD06_LiDAR.h"
#include "bot_config.h"

namespace bot {

extern Adafruit_NeoPixel status_led;
extern HardwareSerial lidar_serial;
extern lidar::Reader lidar_reader;

extern volatile bool activity_flag;
extern volatile char mode;
extern volatile char direction;
extern volatile unsigned long last_cmd_time;

extern uint32_t led_base_color;
extern unsigned long led_flash_end;
extern unsigned long last_lidar_status_ms;

extern LidarState lidar_state;
extern StuckTracker stuck_tracker;
extern WanderAction wander_next_action;
extern unsigned long wander_deadline_ms;
extern bool g_dodging;
extern unsigned long g_dodge_end_ms;
extern bool g_unsticking;
extern unsigned long g_unstuck_end_ms;
extern bool g_wall_follow_left;
extern unsigned long last_telemetry_ms;
extern uint16_t telemetry_frame_id;
extern bool controller_peer_known;
extern uint8_t controller_peer_addr[6];

} // namespace bot
