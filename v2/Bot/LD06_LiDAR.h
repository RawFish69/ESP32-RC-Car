/*
 * LD06_LiDAR  —  Arduino library for the LD06 LiDAR sensor
 *
 * Quick start
 * -----------
 *   #include <LD06_LiDAR.h>
 *
 *   HardwareSerial lidar_serial(1);
 *   lidar::Reader  reader(lidar_serial);
 *
 *   void setup() { reader.begin(rx_pin, tx_pin, 230400); }
 *
 *   void loop() {
 *     if (reader.read_scan()) {
 *       const lidar::ScanFrame &scan = reader.latest_scan();
 *       // use scan.points[], scan.valid_point_count, etc.
 *     }
 *   }
 *
 * Key types (from lidar_data.h)
 * ------------------------------
 *   lidar::ScanPoint   angle_deg, distance_mm, x_mm, y_mm, intensity, valid
 *   lidar::ScanFrame   points[], point_count, valid_point_count, speed_raw
 *
 * Key methods (from lidar_reader.h)
 * -----------------------------------
 *   reader.begin(rx, tx, baud)   start UART and begin parsing
 *   reader.read_scan()           call every loop(); returns true when a full scan is ready
 *   reader.latest_scan()         the most recent complete ScanFrame
 */

#pragma once

#include "lidar_data.h"
#include "lidar_reader.h"
