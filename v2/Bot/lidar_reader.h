#pragma once

#include <Arduino.h>

#include "lidar_data.h"

namespace lidar
{

class Reader
{
public:
  explicit Reader(HardwareSerial &serial_port);

  void begin(int rx_pin, int tx_pin, uint32_t baud_rate);
  bool read_scan();
  void print_packet_summary(Stream &stream) const;

  const PacketSummary &last_packet() const;
  const ScanFrame &latest_scan() const;
  uint32_t packets_seen() const;

private:
  static uint16_t read_le_u16(const uint8_t *data);
  static float normalize_angle(float angle_deg);
  static bool decode_packet(const uint8_t *packet, PacketSummary &summary);

  void clear_scan_frame(ScanFrame &frame);
  void append_point_to_current_scan(const PacketSummary &summary,
                                    const ScanPoint &point);
  void publish_current_scan();
  bool process_packet(const PacketSummary &summary);

  HardwareSerial &serial_;
  uint8_t packet_buffer_[kPacketSize]{};
  size_t packet_index_ = 0;
  uint8_t computed_crc_ = 0;
  PacketSummary last_packet_{};
  ScanFrame scan_buffers_[2]{};
  ScanFrame *current_scan_ = &scan_buffers_[0];
  ScanFrame *latest_scan_ = &scan_buffers_[1];
  bool scan_initialized_ = false;
  bool have_last_physical_angle_ = false;
  float last_physical_angle_deg_ = 0.0f;
  float start_physical_angle_deg_ = 0.0f;
  uint32_t packets_seen_ = 0;
  uint32_t crc_fail_count_ = 0;
};

} // namespace lidar
