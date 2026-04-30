#include "bot_state.h"

namespace bot {

Adafruit_NeoPixel status_led(1, kLedPin, NEO_GRB + NEO_KHZ800);
HardwareSerial lidar_serial(1);
lidar::Reader lidar_reader(lidar_serial);

volatile bool activity_flag = false;
volatile char mode = 'x';
volatile char direction = 'x';
volatile unsigned long last_cmd_time = 0;

uint32_t led_base_color = 0;
unsigned long led_flash_end = 0;
unsigned long last_lidar_status_ms = 0;

LidarState lidar_state;
StuckTracker stuck_tracker;
WanderAction wander_next_action = kDoForward;
unsigned long wander_deadline_ms = 0;
bool g_dodging = false;
unsigned long g_dodge_end_ms = 0;
bool g_unsticking = false;
unsigned long g_unstuck_end_ms = 0;
bool g_wall_follow_left = true;
unsigned long last_telemetry_ms = 0;
uint16_t telemetry_frame_id = 0;
bool controller_peer_known = false;
uint8_t controller_peer_addr[6]{};

} // namespace bot
