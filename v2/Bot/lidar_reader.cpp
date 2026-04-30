#include "lidar_reader.h"

#include <math.h>

namespace lidar
{
namespace
{

constexpr float kSensorYawOffsetDeg = 0.0f;
constexpr bool kMirrorScanYAxis = true;

constexpr uint8_t kCrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68,
    0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d,
    0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f,
    0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a,
    0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
    0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73,
    0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5,
    0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39,
    0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb,
    0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7,
    0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a,
    0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d,
    0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a,
    0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a,
    0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
    0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76,
    0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb,
    0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce,
    0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe,
    0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9,
    0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f,
    0x32, 0xe5, 0xa8};

} // namespace

Reader::Reader(HardwareSerial &serial_port) : serial_(serial_port)
{
  clear_scan_frame(scan_buffers_[0]);
  clear_scan_frame(scan_buffers_[1]);
}

void Reader::begin(int rx_pin, int tx_pin, uint32_t baud_rate)
{
  serial_.begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
}

uint16_t Reader::read_le_u16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

float Reader::normalize_angle(float angle_deg)
{
  while (angle_deg < 0.0f)
  {
    angle_deg += 360.0f;
  }
  while (angle_deg >= 360.0f)
  {
    angle_deg -= 360.0f;
  }
  return angle_deg;
}

bool Reader::decode_packet(const uint8_t *packet, PacketSummary &summary)
{
  if (packet[0] != kPacketHeader || packet[1] != kPacketLength)
  {
    return false;
  }

  summary.speed_raw = read_le_u16(packet + 2);
  summary.start_angle_deg = read_le_u16(packet + 4) / 100.0f;
  summary.end_angle_deg = read_le_u16(packet + 42) / 100.0f;
  summary.timestamp_ms = read_le_u16(packet + 44);
  summary.valid_points = 0;

  float span_deg = summary.end_angle_deg - summary.start_angle_deg;
  if (span_deg < 0.0f)
  {
    span_deg += 360.0f;
  }

  const float step_deg = span_deg / static_cast<float>(kPointsPerPacket);
  if (step_deg <= 0.0f || step_deg > kMaxAngleStepDeg)
  {
    return false;
  }

  for (size_t i = 0; i < kPointsPerPacket; ++i)
  {
    const size_t offset = 6 + (i * 3);
    const uint16_t distance_mm = read_le_u16(packet + offset);
    const uint8_t intensity = packet[offset + 2];
    const float physical_angle_deg =
        normalize_angle(summary.start_angle_deg +
                        ((static_cast<float>(i) + 0.5f) * step_deg));
    const float angle_deg =
        normalize_angle(360.0f - physical_angle_deg + kSensorYawOffsetDeg);
    const float angle_rad = angle_deg * DEG_TO_RAD;

    ScanPoint point;
    point.angle_deg = angle_deg;
    point.distance_mm = distance_mm;
    point.intensity = intensity;
    point.x_mm = static_cast<int16_t>(lroundf(distance_mm * cosf(angle_rad)));
    point.y_mm = static_cast<int16_t>(
        lroundf((kMirrorScanYAxis ? 1.0f : -1.0f) * distance_mm * sinf(angle_rad)));
    point.valid = distance_mm > 0;
    summary.points[i] = point;

    if (point.valid)
    {
      ++summary.valid_points;
    }
  }

  return true;
}

void Reader::clear_scan_frame(ScanFrame &frame)
{
  frame.speed_raw = 0;
  frame.timestamp_ms = 0;
  frame.point_count = 0;
  frame.valid_point_count = 0;
  frame.crc_fail_count = crc_fail_count_;
}

void Reader::append_point_to_current_scan(const PacketSummary &summary,
                                          const ScanPoint &point)
{
  current_scan_->speed_raw = summary.speed_raw;
  current_scan_->timestamp_ms = summary.timestamp_ms;
  current_scan_->crc_fail_count = crc_fail_count_;

  if (current_scan_->point_count >= kMaxPointsPerScan)
  {
    return;
  }

  current_scan_->points[current_scan_->point_count++] = point;
  if (point.valid)
  {
    ++current_scan_->valid_point_count;
  }
}

void Reader::publish_current_scan()
{
  latest_scan_ = current_scan_;
  latest_scan_->crc_fail_count = crc_fail_count_;

  current_scan_ =
      (current_scan_ == &scan_buffers_[0]) ? &scan_buffers_[1] : &scan_buffers_[0];
  clear_scan_frame(*current_scan_);
}

bool Reader::process_packet(const PacketSummary &summary)
{
  bool new_scan_ready = false;

  for (size_t i = 0; i < kPointsPerPacket; ++i)
  {
    const ScanPoint &point = summary.points[i];
    const float physical_angle_deg = normalize_angle(360.0f - point.angle_deg);

    if (!have_last_physical_angle_)
    {
      have_last_physical_angle_ = true;
      last_physical_angle_deg_ = physical_angle_deg;
      start_physical_angle_deg_ = physical_angle_deg;
    }

    if (physical_angle_deg < last_physical_angle_deg_)
    {
      if (!scan_initialized_)
      {
        scan_initialized_ = true;
        clear_scan_frame(*current_scan_);
      }
      else if ((last_physical_angle_deg_ - start_physical_angle_deg_) > 340.0f &&
               current_scan_->point_count > 0)
      {
        publish_current_scan();
        new_scan_ready = true;
      }

      start_physical_angle_deg_ = physical_angle_deg;
    }

    last_physical_angle_deg_ = physical_angle_deg;

    if (!scan_initialized_)
    {
      continue;
    }

    append_point_to_current_scan(summary, point);
  }

  return new_scan_ready;
}

bool Reader::read_scan()
{
  bool new_scan_ready = false;

  while (serial_.available() > 0)
  {
    const uint8_t byte_in = static_cast<uint8_t>(serial_.read());

    if (packet_index_ == 0)
    {
      if (byte_in != kPacketHeader)
      {
        continue;
      }

      packet_buffer_[packet_index_] = byte_in;
      computed_crc_ = kCrcTable[byte_in];
      ++packet_index_;
      continue;
    }

    if (packet_index_ == 1 && byte_in != kPacketLength)
    {
      packet_index_ = 0;
      computed_crc_ = 0;
      if (byte_in == kPacketHeader)
      {
        packet_buffer_[packet_index_] = byte_in;
        computed_crc_ = kCrcTable[byte_in];
        ++packet_index_;
      }
      continue;
    }

    packet_buffer_[packet_index_] = byte_in;

    if (packet_index_ < (kPacketSize - 1))
    {
      computed_crc_ = kCrcTable[computed_crc_ ^ byte_in];
      ++packet_index_;
      continue;
    }

    const uint8_t expected_crc = computed_crc_;
    packet_index_ = 0;
    computed_crc_ = 0;

    if (expected_crc != byte_in)
    {
      ++crc_fail_count_;
      continue;
    }

    PacketSummary summary;
    if (decode_packet(packet_buffer_, summary))
    {
      last_packet_ = summary;
      ++packets_seen_;
      if (process_packet(summary))
      {
        new_scan_ready = true;
      }
    }
  }

  return new_scan_ready;
}

void Reader::print_packet_summary(Stream &stream) const
{
  stream.printf("packets=%lu speed=%u start=%.2f end=%.2f valid=%u ts=%u crc_fail=%lu\n",
                static_cast<unsigned long>(packets_seen_),
                last_packet_.speed_raw,
                last_packet_.start_angle_deg,
                last_packet_.end_angle_deg,
                static_cast<unsigned>(last_packet_.valid_points),
                last_packet_.timestamp_ms,
                static_cast<unsigned long>(crc_fail_count_));

  for (size_t i = 0; i < kPointsPerPacket; ++i)
  {
    const ScanPoint &point = last_packet_.points[i];
    if (!point.valid)
    {
      continue;
    }

    stream.printf("  angle=%7.2f deg dist=%5u mm intensity=%3u x=%6d y=%6d\n",
                  point.angle_deg,
                  point.distance_mm,
                  point.intensity,
                  point.x_mm,
                  point.y_mm);
  }
}

const PacketSummary &Reader::last_packet() const
{
  return last_packet_;
}

const ScanFrame &Reader::latest_scan() const
{
  return *latest_scan_;
}

uint32_t Reader::packets_seen() const
{
  return packets_seen_;
}

} // namespace lidar
