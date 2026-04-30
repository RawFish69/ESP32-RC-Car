#pragma once

#include "bot_config.h"

namespace bot {

void reset_wander_state();
void reset_stuck_tracker();
void wander_avoidance(const WanderConfig &cfg, bool emergency);
void wander(const WanderConfig &cfg);
void basic_wander();
void try_active_dodge(const WanderConfig &cfg);
bool maybe_start_unstuck();
void activate_mode(char new_mode);

} // namespace bot
