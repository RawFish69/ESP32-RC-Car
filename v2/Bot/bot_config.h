#pragma once

#include <Arduino.h>

namespace bot {

// TB6612FNG wiring for the bot drive base.
// If your SLAM rover routes STBY to a GPIO, set that pin below.
// Leave it at -1 if STBY is tied high on the driver board.
constexpr int kLeftPwmPin = 4;       // PWMA
constexpr int kLeftMotorIn1Pin = 6;  // AIN1
constexpr int kLeftMotorIn2Pin = 5;  // AIN2
constexpr int kRightMotorIn1Pin = 7; // BIN1
constexpr int kRightMotorIn2Pin = 8; // BIN2
constexpr int kRightPwmPin = 10;     // PWMB
constexpr int kMotorStandbyPin = -1;
constexpr int kLedPin = 2;

// ESP32-C3 UART for LD06.
constexpr int kLidarTxPin = 1;
constexpr int kLidarRxPin = 0;
constexpr uint32_t kLidarBaud = 230400;
constexpr unsigned long kLidarFreshMs = 700;
constexpr unsigned long kTelemetryIntervalMs = 120;

constexpr uint8_t kTelemetryMagic = 0xA5;
constexpr uint8_t kTelemetryVersion = 1;
constexpr uint8_t kTelemetryTypeScanChunk = 1;
constexpr uint8_t kTelemetryTypeMotion = 2;
constexpr uint8_t kTelemetryPointsPerChunk = 48;
constexpr uint8_t kTelemetryMaxPoints = 96;

constexpr uint8_t kLedcBits = 14;
constexpr uint32_t kLedcMax = (1UL << kLedcBits) - 1;
constexpr uint32_t kLedcFreqHz = 30;

// Flip either side to -1 if that motor spins backward with positive speed.
constexpr int kLeftMotorPolarity = 1;
constexpr int kRightMotorPolarity = 1;

// Teleop fixed speeds (used by forward/backward/turn/spin helpers for manual control).
constexpr int kDriveSpeed = 170;
constexpr int kTurnFast = 200;
constexpr int kTurnSlow = 60;
constexpr int kSpinSpeed = 180;
constexpr int kReverseSpeed = 130;

constexpr unsigned long kTeleopTimeoutMs = 800;
constexpr unsigned long kWanderFwdMinMs = 700;
constexpr unsigned long kWanderFwdMaxMs = 2800;
constexpr unsigned long kWanderTurnMinMs = 350;
constexpr unsigned long kWanderTurnMaxMs = 1000;
constexpr unsigned long kAvoidReverseMs = 220;
constexpr unsigned long kAvoidArcMinMs = 220;
constexpr unsigned long kAvoidArcMaxMs = 650;
constexpr unsigned long kAvoidSpinMinMs = 350;
constexpr unsigned long kAvoidSpinMaxMs = 850;
constexpr unsigned long kStuckDetectMs = 650;
constexpr unsigned long kUnstuckReverseMs = 240;
constexpr unsigned long kUnstuckPivotMinMs = 500;
constexpr unsigned long kUnstuckPivotMaxMs = 900;
constexpr unsigned long kUnstuckSpinMinMs = 500;
constexpr unsigned long kUnstuckSpinMaxMs = 950;
constexpr unsigned long kLidarStatusIntervalMs = 3000;

constexpr uint16_t kLidarIgnoreNearMm = 40;
constexpr uint16_t kLidarDefaultOpenMm = 5000;
constexpr int kContactMinForwardMm = 0;
constexpr int kContactHalfWidthMm = 100;
constexpr uint16_t kContactEmergencyMm = 85;
constexpr int kFrontMinForwardMm = 35;
constexpr int kRearMinBackwardMm = 70;
constexpr int kRearCornerMinBackwardMm = 70;
constexpr int kRearCornerMinLateralMm = 80;
constexpr int kFrontHalfWidthMm = 130;
constexpr int kRearHalfWidthMm = 130;
// Only use forward-side sectors for avoidance decisions so the rover's rear
// footprint is not mistaken for a wall.
constexpr int kSideMinForwardMm = 0;
constexpr int kSideMinLateralMm = 80;
constexpr uint16_t kRearBlockedMm = 60;
constexpr uint16_t kWanderRearCornerMm = 140;
constexpr uint16_t kWanderSideEmergencyMm = 120;
constexpr uint16_t kStuckNearFrontMm = 170;
constexpr uint16_t kStuckNearRearMm = 90;
constexpr uint16_t kStuckNearSideMm = 160;
constexpr uint16_t kStuckDeltaMm = 35;

// Legacy active-dodge thresholds kept for reference; active dodge is currently disabled.
constexpr uint16_t kDodgeTriggerMm = 500;
constexpr uint16_t kDodgeApproachMm = 180;
constexpr unsigned long kDodgeDurationMs = 450;

struct WanderConfig
{
  int drive_speed;
  int turn_fast;
  int turn_slow;
  int spin_speed;
  int reverse_speed;
  bool reverse_on_emergency;
  uint16_t emergency_front_mm;
  uint16_t caution_front_mm;
  uint16_t preferred_clear_mm;
};

// Basic wander config.
// Additional wander profiles (modes 2, 3, 5) are not included in this release.
constexpr WanderConfig kBasicWander = {220, 240, 90, 230, 180, true, 280, 500, 700};

// ── Optional: IMU ──────────────────────────────────────────────────────────
// Uncomment to enable IMU-assisted heading and tilt compensation.
// Useful for SLAM and smoother navigation.
//
// #define BOT_HAS_IMU
// constexpr int kImuSdaPin  = 3;
// constexpr int kImuSclPin  = 4;
// constexpr float kImuAlpha = 0.98f;  // complementary filter weight (gyro vs accel)

// ── Optional: Motor Encoders + PID ─────────────────────────────────────────
// Uncomment to enable closed-loop speed control via motor encoders.
// Required for odometry and accurate SLAM.
//
// #define BOT_HAS_ENCODERS
// constexpr int kLeftEncoderPin   = 9;
// constexpr int kRightEncoderPin  = 11;
// constexpr int kEncoderPpr       = 20;   // pulses per revolution
// constexpr float kWheelDiameterM = 0.065f;
// constexpr float kPidKp = 1.2f;
// constexpr float kPidKi = 0.3f;
// constexpr float kPidKd = 0.05f;

enum WanderAction
{
  kDoForward,
  kDoTurn,
};

struct LidarState
{
  bool have_scan = false;
  unsigned long last_scan_ms = 0;
  uint16_t contact_min_mm = 0;
  uint16_t front_min_mm = 0;
  uint16_t rear_min_mm = 0;
  uint16_t rear_left_min_mm = 0;
  uint16_t rear_right_min_mm = 0;
  uint16_t left_min_mm = 0;
  uint16_t right_min_mm = 0;
  uint16_t prev_left_mm = 0;
  uint16_t prev_right_mm = 0;
  uint16_t valid_points = 0;
  uint16_t scan_points = 0;
  uint32_t crc_fail_count = 0;
  uint32_t packets_seen = 0;
};

struct StuckTracker
{
  bool armed = false;
  char direction = 'x';
  unsigned long start_ms = 0;
  uint16_t front_mm = 0;
  uint16_t rear_mm = 0;
  uint16_t left_mm = 0;
  uint16_t right_mm = 0;
};

struct __attribute__((packed)) TelemetryHeader
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  uint16_t frame_id;
  uint8_t chunk_index;
  uint8_t chunk_count;
  uint8_t point_count;
  uint8_t total_points;
};

struct __attribute__((packed)) TelemetryPoint
{
  int16_t x_mm;
  int16_t y_mm;
  uint8_t intensity;
};

struct __attribute__((packed)) MotionTelemetry
{
  uint8_t magic;
  uint8_t version;
  uint8_t type;
  char mode;
  char direction;
  uint8_t flags;
};

} // namespace bot
