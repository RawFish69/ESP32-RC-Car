#pragma once

#include <Arduino.h>

namespace lidar
{

constexpr uint8_t kPacketHeader = 0x54;
constexpr uint8_t kPacketLength = 0x2C;
constexpr size_t kPointsPerPacket = 12;
constexpr size_t kPacketSize = 47;
constexpr size_t kMaxPointsPerScan = 1200;
constexpr float kMaxAngleStepDeg = 5.0f;

struct ScanPoint
{
  float angle_deg = 0.0f;
  uint16_t distance_mm = 0;
  uint8_t intensity = 0;
  int16_t x_mm = 0;
  int16_t y_mm = 0;
  bool valid = false;
};

struct PacketSummary
{
  uint16_t speed_raw = 0;
  float start_angle_deg = 0.0f;
  float end_angle_deg = 0.0f;
  uint16_t timestamp_ms = 0;
  uint16_t valid_points = 0;
  ScanPoint points[kPointsPerPacket]{};
};

struct ScanFrame
{
  uint16_t speed_raw = 0;
  uint16_t timestamp_ms = 0;
  uint16_t point_count = 0;
  uint16_t valid_point_count = 0;
  uint32_t crc_fail_count = 0;
  ScanPoint points[kMaxPointsPerScan]{};
};

} // namespace lidar
