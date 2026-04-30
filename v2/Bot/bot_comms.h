#pragma once

#include <esp_now.h>

namespace bot {

void on_data_recv(const esp_now_recv_info *info, const uint8_t *data, int len);
void setup_espnow();
void handle_usb_serial();

} // namespace bot
