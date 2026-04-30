#pragma once

#include <Arduino.h>

#include "LD06_LiDAR.h"
#include "bot_config.h"

namespace bot {

bool lidar_is_fresh();
bool should_turn_left();
void refresh_lidar_state(const lidar::ScanFrame &scan);
void update_lidar();
void print_lidar_status();
void maybe_report_lidar();
void setup_lidar();
void select_wall_follow_side();

} // namespace bot
