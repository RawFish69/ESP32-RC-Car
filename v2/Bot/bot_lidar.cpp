#include "bot_lidar.h"

#include <esp_now.h>
#include <string.h>

#include "bot_behaviors.h"
#include "bot_state.h"

namespace bot {
namespace {

uint16_t choose_min_distance(uint16_t current_mm, uint16_t candidate_mm)
{
  if (candidate_mm == 0)
  {
    return current_mm;
  }
  if (current_mm == 0 || candidate_mm < current_mm)
  {
    return candidate_mm;
  }
  return current_mm;
}

uint16_t clearance_or_default(uint16_t distance_mm)
{
  return distance_mm == 0 ? kLidarDefaultOpenMm : distance_mm;
}

} // namespace

bool lidar_is_fresh()
{
  return lidar_state.have_scan &&
         (millis() - lidar_state.last_scan_ms) <= kLidarFreshMs;
}

bool should_turn_left()
{
  const uint16_t left_clear = clearance_or_default(lidar_state.left_min_mm);
  const uint16_t right_clear = clearance_or_default(lidar_state.right_min_mm);
  if (left_clear == right_clear)
  {
    return random(2) == 0;
  }
  return left_clear > right_clear;
}

void select_wall_follow_side()
{
  g_wall_follow_left = (lidar_state.right_min_mm == 0) ||
                       (lidar_state.left_min_mm > 0 &&
                        lidar_state.left_min_mm <= lidar_state.right_min_mm);
}

void refresh_lidar_state(const lidar::ScanFrame &scan)
{
  lidar_state.prev_left_mm = lidar_state.left_min_mm;
  lidar_state.prev_right_mm = lidar_state.right_min_mm;
  lidar_state.have_scan = scan.valid_point_count > 0;
  lidar_state.last_scan_ms = millis();
  lidar_state.contact_min_mm = 0;
  lidar_state.front_min_mm = 0;
  lidar_state.rear_min_mm = 0;
  lidar_state.rear_left_min_mm = 0;
  lidar_state.rear_right_min_mm = 0;
  lidar_state.left_min_mm = 0;
  lidar_state.right_min_mm = 0;
  lidar_state.valid_points = scan.valid_point_count;
  lidar_state.scan_points = scan.point_count;
  lidar_state.crc_fail_count = scan.crc_fail_count;
  lidar_state.packets_seen = lidar_reader.packets_seen();

  for (uint16_t i = 0; i < scan.point_count; ++i)
  {
    const lidar::ScanPoint &point = scan.points[i];
    if (!point.valid || point.distance_mm < kLidarIgnoreNearMm)
    {
      continue;
    }

    if (point.x_mm >= kContactMinForwardMm &&
        abs(point.y_mm) <= kContactHalfWidthMm)
    {
      lidar_state.contact_min_mm =
          choose_min_distance(lidar_state.contact_min_mm, point.distance_mm);
    }

    if (point.x_mm >= kFrontMinForwardMm && abs(point.y_mm) <= kFrontHalfWidthMm)
    {
      lidar_state.front_min_mm =
          choose_min_distance(lidar_state.front_min_mm, point.distance_mm);
    }
    else if (point.x_mm <= -kRearMinBackwardMm && abs(point.y_mm) <= kRearHalfWidthMm)
    {
      lidar_state.rear_min_mm =
          choose_min_distance(lidar_state.rear_min_mm, point.distance_mm);
    }

    if (point.x_mm <= -kRearCornerMinBackwardMm &&
        point.y_mm >= kRearCornerMinLateralMm)
    {
      lidar_state.rear_left_min_mm =
          choose_min_distance(lidar_state.rear_left_min_mm, point.distance_mm);
    }
    else if (point.x_mm <= -kRearCornerMinBackwardMm &&
             point.y_mm <= -kRearCornerMinLateralMm)
    {
      lidar_state.rear_right_min_mm =
          choose_min_distance(lidar_state.rear_right_min_mm, point.distance_mm);
    }

    if (point.x_mm >= kSideMinForwardMm && point.y_mm >= kSideMinLateralMm)
    {
      lidar_state.left_min_mm =
          choose_min_distance(lidar_state.left_min_mm, point.distance_mm);
    }
    else if (point.x_mm >= kSideMinForwardMm &&
             point.y_mm <= -kSideMinLateralMm)
    {
      lidar_state.right_min_mm =
          choose_min_distance(lidar_state.right_min_mm, point.distance_mm);
    }
  }
}

void update_lidar()
{
  if (lidar_reader.read_scan())
  {
    const lidar::ScanFrame &scan = lidar_reader.latest_scan();
    refresh_lidar_state(scan);

    const unsigned long now = millis();
    if (controller_peer_known && (now - last_telemetry_ms) >= kTelemetryIntervalMs)
    {
      last_telemetry_ms = now;
      ++telemetry_frame_id;

      TelemetryPoint sampled_points[kTelemetryMaxPoints]{};
      uint8_t sampled_count = 0;

      if (scan.valid_point_count > 0)
      {
        const uint16_t desired_points =
            (scan.valid_point_count < kTelemetryMaxPoints)
                ? scan.valid_point_count
                : static_cast<uint16_t>(kTelemetryMaxPoints);
        uint16_t valid_index = 0;
        uint8_t selected = 0;

        for (uint16_t i = 0; i < scan.point_count && selected < desired_points; ++i)
        {
          const lidar::ScanPoint &point = scan.points[i];
          if (!point.valid)
          {
            continue;
          }

          const uint16_t bucket =
              static_cast<uint32_t>(valid_index) * desired_points / scan.valid_point_count;
          if (bucket == selected)
          {
            sampled_points[selected].x_mm = point.x_mm;
            sampled_points[selected].y_mm = point.y_mm;
            sampled_points[selected].intensity = point.intensity;
            ++selected;
          }

          ++valid_index;
        }

        sampled_count = selected;
      }

      if (sampled_count > 0)
      {
        const uint8_t chunk_count =
            (sampled_count + kTelemetryPointsPerChunk - 1) / kTelemetryPointsPerChunk;

        for (uint8_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index)
        {
          const uint8_t point_offset = chunk_index * kTelemetryPointsPerChunk;
          const uint8_t remaining = sampled_count - point_offset;
          const uint8_t points_in_chunk =
              (remaining < kTelemetryPointsPerChunk) ? remaining : kTelemetryPointsPerChunk;
          const size_t packet_size =
              sizeof(TelemetryHeader) + points_in_chunk * sizeof(TelemetryPoint);
          uint8_t packet_buffer[sizeof(TelemetryHeader) +
                                kTelemetryPointsPerChunk * sizeof(TelemetryPoint)]{};

          TelemetryHeader header{
              kTelemetryMagic,
              kTelemetryVersion,
              kTelemetryTypeScanChunk,
              telemetry_frame_id,
              chunk_index,
              chunk_count,
              points_in_chunk,
              sampled_count,
          };

          memcpy(packet_buffer, &header, sizeof(header));
          memcpy(packet_buffer + sizeof(header),
                 &sampled_points[point_offset],
                 points_in_chunk * sizeof(TelemetryPoint));
          esp_now_send(controller_peer_addr, packet_buffer, packet_size);
        }
      }

      const MotionTelemetry motion_packet{
          kTelemetryMagic,
          kTelemetryVersion,
          kTelemetryTypeMotion,
          static_cast<char>(mode),
          static_cast<char>(direction),
          static_cast<uint8_t>(((g_dodging || g_unsticking) ? 0x01 : 0x00) |
                               (g_wall_follow_left ? 0x02 : 0x00)),
      };
      esp_now_send(controller_peer_addr,
                   reinterpret_cast<const uint8_t *>(&motion_packet),
                   sizeof(motion_packet));
    }
  }
}

void print_lidar_status()
{
  Serial.printf("Lidar packets=%lu scan_points=%u valid=%u contact=%u front=%u rear=%u rear_left=%u rear_right=%u left=%u right=%u crc_fail=%lu\n",
                static_cast<unsigned long>(lidar_state.packets_seen),
                static_cast<unsigned>(lidar_state.scan_points),
                static_cast<unsigned>(lidar_state.valid_points),
                static_cast<unsigned>(lidar_state.contact_min_mm),
                static_cast<unsigned>(lidar_state.front_min_mm),
                static_cast<unsigned>(lidar_state.rear_min_mm),
                static_cast<unsigned>(lidar_state.rear_left_min_mm),
                static_cast<unsigned>(lidar_state.rear_right_min_mm),
                static_cast<unsigned>(lidar_state.left_min_mm),
                static_cast<unsigned>(lidar_state.right_min_mm),
                static_cast<unsigned long>(lidar_state.crc_fail_count));
}

void maybe_report_lidar()
{
  if (!lidar_is_fresh())
  {
    return;
  }

  const unsigned long now = millis();
  if (now - last_lidar_status_ms >= kLidarStatusIntervalMs)
  {
    last_lidar_status_ms = now;
    print_lidar_status();
  }
}

void setup_lidar()
{
  lidar_reader.begin(kLidarRxPin, kLidarTxPin, kLidarBaud);
  Serial.printf("LD06 ready on Serial1 RX=%d TX=%d baud=%lu\n",
                kLidarRxPin,
                kLidarTxPin,
                static_cast<unsigned long>(kLidarBaud));
}

} // namespace bot
