#include "bot_behaviors.h"

#include "bot_lidar.h"
#include "bot_led.h"
#include "bot_motor.h"
#include "bot_state.h"

namespace bot {
namespace {

uint16_t sector_delta(uint16_t now_mm, uint16_t ref_mm)
{
  if (now_mm == 0 && ref_mm == 0)
  {
    return 0;
  }
  if (now_mm == 0 || ref_mm == 0)
  {
    return 10000;
  }
  return (now_mm > ref_mm) ? (now_mm - ref_mm) : (ref_mm - now_mm);
}

bool is_near(uint16_t distance_mm, uint16_t threshold_mm)
{
  return distance_mm > 0 && distance_mm <= threshold_mm;
}

bool start_wander_escape(char dir, unsigned long min_ms, unsigned long max_ms)
{
  switch (dir)
  {
    case 'q':
      drive(-kBasicWander.spin_speed, kBasicWander.spin_speed);
      direction = 'q';
      break;
    case 'e':
      drive(kBasicWander.spin_speed, -kBasicWander.spin_speed);
      direction = 'e';
      break;
    case 'a':
      drive(0, kBasicWander.turn_fast);
      direction = 'a';
      break;
    case 'd':
      drive(kBasicWander.turn_fast, 0);
      direction = 'd';
      break;
    default:
      return false;
  }

  wander_deadline_ms = millis() + random(min_ms, max_ms);
  wander_next_action = kDoForward;
  return true;
}

uint16_t front_reaction_distance_mm()
{
  if (lidar_state.front_min_mm > 0)
  {
    return lidar_state.front_min_mm;
  }
  return lidar_state.contact_min_mm;
}

void start_unstuck_escape()
{
  const uint16_t front_mm = front_reaction_distance_mm();
  const bool front_blocked = is_near(front_mm, kStuckNearFrontMm);
  const bool hard_contact = is_near(front_mm, kContactEmergencyMm);
  const bool rear_blocked = is_near(lidar_state.rear_min_mm, kRearBlockedMm);
  const bool turn_left = should_turn_left();

  if (front_blocked && !rear_blocked)
  {
    drive(-kReverseSpeed, -kReverseSpeed);
    direction = 's';
    delay(hard_contact ? (kUnstuckReverseMs + 180UL) : kUnstuckReverseMs);
  }

  if (front_blocked)
  {
    if (turn_left)
    {
      drive(-kSpinSpeed, kSpinSpeed);
      direction = 'q';
    }
    else
    {
      drive(kSpinSpeed, -kSpinSpeed);
      direction = 'e';
    }
    g_unstuck_end_ms = millis() + random(kUnstuckSpinMinMs, kUnstuckSpinMaxMs);
  }
  else
  {
    if (turn_left)
    {
      drive(0, kTurnFast);
      direction = 'a';
    }
    else
    {
      drive(kTurnFast, 0);
      direction = 'd';
    }
    g_unstuck_end_ms = millis() + random(kUnstuckPivotMinMs, kUnstuckPivotMaxMs);
  }

  g_unsticking = true;
  reset_wander_state();
  reset_stuck_tracker();
  trigger_activity();
}

} // namespace

void reset_stuck_tracker()
{
  stuck_tracker.armed = false;
  stuck_tracker.direction = 'x';
  stuck_tracker.start_ms = 0;
  stuck_tracker.front_mm = 0;
  stuck_tracker.rear_mm = 0;
  stuck_tracker.left_mm = 0;
  stuck_tracker.right_mm = 0;
}

void reset_wander_state()
{
  wander_next_action = kDoForward;
  wander_deadline_ms = 0;
}

void wander_avoidance(const WanderConfig &cfg, bool emergency)
{
  const bool go_left = should_turn_left();
  if (emergency)
  {
    const uint16_t front_mm = front_reaction_distance_mm();
    const bool rear_blocked = lidar_state.rear_min_mm > 0 &&
                              lidar_state.rear_min_mm <= kRearBlockedMm;
    if (cfg.reverse_on_emergency && !rear_blocked)
    {
      const bool hard_contact = front_mm > 0 && front_mm <= kContactEmergencyMm;
      drive(-cfg.reverse_speed, -cfg.reverse_speed);
      delay(hard_contact ? (kAvoidReverseMs + 180UL) : (kAvoidReverseMs + 80UL));
      if (go_left)
      {
        drive(-cfg.spin_speed, cfg.spin_speed);
        direction = 'q';
      }
      else
      {
        drive(cfg.spin_speed, -cfg.spin_speed);
        direction = 'e';
      }
      wander_deadline_ms = millis() + random(kAvoidSpinMinMs + 120UL, kAvoidSpinMaxMs + 180UL);
    }
    else
    {
      // In non-reverse modes, prefer a pivot-away over a forward arc so the bot
      // stops pushing into the obstacle while still avoiding reverse-heavy motion.
      if (go_left)
      {
        drive(0, cfg.turn_fast);
        direction = 'a';
      }
      else
      {
        drive(cfg.turn_fast, 0);
        direction = 'd';
      }
      wander_deadline_ms = millis() + random(kAvoidArcMinMs, kAvoidArcMaxMs);
    }
  }
  else
  {
    if (go_left)
    {
      drive(cfg.turn_slow, cfg.turn_fast);
      direction = 'a';
    }
    else
    {
      drive(cfg.turn_fast, cfg.turn_slow);
      direction = 'd';
    }
    wander_deadline_ms = millis() + random(kAvoidArcMinMs, kAvoidArcMaxMs);
  }
  wander_next_action = kDoForward;
}

void basic_wander()
{
  if (lidar_is_fresh())
  {
    const uint16_t front_mm = front_reaction_distance_mm();
    const bool front_emergency =
        front_mm > 0 && front_mm <= kBasicWander.emergency_front_mm;
    const bool left_close =
        is_near(lidar_state.left_min_mm, kWanderSideEmergencyMm);
    const bool right_close =
        is_near(lidar_state.right_min_mm, kWanderSideEmergencyMm);
    const bool rear_left_close =
        is_near(lidar_state.rear_left_min_mm, kWanderRearCornerMm);
    const bool rear_right_close =
        is_near(lidar_state.rear_right_min_mm, kWanderRearCornerMm);
    const bool rear_close =
        is_near(lidar_state.rear_min_mm, kRearBlockedMm);

    if (front_emergency)
    {
      wander_avoidance(kBasicWander, true);
      return;
    }
    if (rear_left_close && rear_right_close)
    {
      if (start_wander_escape(should_turn_left() ? 'a' : 'd',
                                  kAvoidArcMinMs + 120UL,
                                  kAvoidArcMaxMs + 180UL))
      {
        return;
      }
    }
    if (rear_left_close || rear_close)
    {
      if (rear_left_close &&
          start_wander_escape('d', kAvoidArcMinMs + 120UL, kAvoidArcMaxMs + 180UL))
      {
        return;
      }
    }
    if (rear_right_close || rear_close)
    {
      if (rear_right_close &&
          start_wander_escape('a', kAvoidArcMinMs + 120UL, kAvoidArcMaxMs + 180UL))
      {
        return;
      }
    }
    if (rear_close)
    {
      if (start_wander_escape(should_turn_left() ? 'a' : 'd',
                                  kAvoidArcMinMs + 120UL,
                                  kAvoidArcMaxMs + 180UL))
      {
        return;
      }
    }
    if (left_close && !right_close)
    {
      if (start_wander_escape('e', kAvoidSpinMinMs, kAvoidSpinMaxMs + 180UL))
      {
        return;
      }
    }
    if (right_close && !left_close)
    {
      if (start_wander_escape('q', kAvoidSpinMinMs, kAvoidSpinMaxMs + 180UL))
      {
        return;
      }
    }
  }

  wander(kBasicWander);
}

void wander(const WanderConfig &cfg)
{
  if (lidar_is_fresh())
  {
    const uint16_t front_mm = front_reaction_distance_mm();
    if (front_mm > 0 && front_mm <= cfg.emergency_front_mm)
    {
      wander_avoidance(cfg, true);
      return;
    }
    if (cfg.caution_front_mm > 0 && front_mm > 0 && front_mm <= cfg.caution_front_mm)
    {
      wander_avoidance(cfg, false);
      return;
    }
  }

  if (millis() < wander_deadline_ms)
  {
    return;
  }

  if (wander_next_action == kDoForward)
  {
    if (cfg.preferred_clear_mm > 0 && lidar_is_fresh() &&
        lidar_state.front_min_mm > 0 &&
        lidar_state.front_min_mm < cfg.preferred_clear_mm)
    {
      wander_avoidance(cfg, false);
      return;
    }
    drive(cfg.drive_speed, cfg.drive_speed);
    direction = 'w';
    wander_deadline_ms = millis() + random(kWanderFwdMinMs, kWanderFwdMaxMs);
    wander_next_action = kDoTurn;
  }
  else
  {
    if (cfg.reverse_on_emergency)
    {
      switch (random(4))
      {
        case 0:
          drive(-cfg.spin_speed, cfg.spin_speed);
          direction = 'q';
          break;
        case 1:
          drive(cfg.spin_speed, -cfg.spin_speed);
          direction = 'e';
          break;
        case 2:
          drive(cfg.turn_slow, cfg.turn_fast);
          direction = 'a';
          break;
        default:
          drive(cfg.turn_fast, cfg.turn_slow);
          direction = 'd';
          break;
      }
    }
    else if (random(2) == 0)
    {
      drive(cfg.turn_slow, cfg.turn_fast);
      direction = 'a';
    }
    else
    {
      drive(cfg.turn_fast, cfg.turn_slow);
      direction = 'd';
    }
    wander_deadline_ms = millis() + random(kWanderTurnMinMs, kWanderTurnMaxMs);
    wander_next_action = kDoForward;
  }
}

void try_active_dodge(const WanderConfig &cfg)
{
  if (!lidar_is_fresh() || g_dodging)
  {
    return;
  }

  const uint16_t front_mm = front_reaction_distance_mm();
  if (front_mm > 0 &&
      (front_mm <= cfg.caution_front_mm || front_mm <= kContactEmergencyMm))
  {
    return;
  }

  const uint16_t left_now = lidar_state.left_min_mm;
  const uint16_t right_now = lidar_state.right_min_mm;
  const uint16_t prev_left = lidar_state.prev_left_mm;
  const uint16_t prev_right = lidar_state.prev_right_mm;

  const bool left_threat = prev_left > 0 && left_now > 0 &&
                           left_now < kDodgeTriggerMm &&
                           (prev_left - left_now) > kDodgeApproachMm;
  const bool right_threat = prev_right > 0 && right_now > 0 &&
                            right_now < kDodgeTriggerMm &&
                            (prev_right - right_now) > kDodgeApproachMm;

  if (!left_threat && !right_threat)
  {
    return;
  }

  if (left_threat && right_threat)
  {
    drive(-cfg.reverse_speed, -cfg.reverse_speed);
    direction = 's';
  }
  else if (left_threat)
  {
    drive(cfg.spin_speed, -cfg.spin_speed);
    direction = 'e';
  }
  else
  {
    drive(-cfg.spin_speed, cfg.spin_speed);
    direction = 'q';
  }

  g_dodging = true;
  g_dodge_end_ms = millis() + kDodgeDurationMs;
  trigger_activity();
}

bool maybe_start_unstuck()
{
  if (!lidar_is_fresh())
  {
    reset_stuck_tracker();
    return false;
  }

  if (mode != '4')
  {
    reset_stuck_tracker();
    return false;
  }

  if (direction == 'x')
  {
    reset_stuck_tracker();
    return false;
  }

  const uint16_t front_now = front_reaction_distance_mm();
  const bool front_near = is_near(front_now, kStuckNearFrontMm);
  const bool rear_near = is_near(lidar_state.rear_min_mm, kStuckNearRearMm);
  const bool left_near = is_near(lidar_state.left_min_mm, kStuckNearSideMm);
  const bool right_near = is_near(lidar_state.right_min_mm, kStuckNearSideMm);
  const uint8_t near_count =
      static_cast<uint8_t>(front_near) +
      static_cast<uint8_t>(rear_near) +
      static_cast<uint8_t>(left_near) +
      static_cast<uint8_t>(right_near);

  if (!stuck_tracker.armed || stuck_tracker.direction != direction)
  {
    stuck_tracker.armed = true;
    stuck_tracker.direction = direction;
    stuck_tracker.start_ms = millis();
    stuck_tracker.front_mm = front_now;
    stuck_tracker.rear_mm = lidar_state.rear_min_mm;
    stuck_tracker.left_mm = lidar_state.left_min_mm;
    stuck_tracker.right_mm = lidar_state.right_min_mm;
    return false;
  }

  if (millis() - stuck_tracker.start_ms < kStuckDetectMs)
  {
    return false;
  }

  const bool pressed_forward =
      (direction == 'w' || direction == 'a' || direction == 'd') && front_near;
  const bool pressed_reverse = direction == 's' && rear_near;
  const bool little_change =
      sector_delta(front_now, stuck_tracker.front_mm) <= kStuckDeltaMm &&
      sector_delta(lidar_state.rear_min_mm, stuck_tracker.rear_mm) <= kStuckDeltaMm &&
      sector_delta(lidar_state.left_min_mm, stuck_tracker.left_mm) <= kStuckDeltaMm &&
      sector_delta(lidar_state.right_min_mm, stuck_tracker.right_mm) <= kStuckDeltaMm;
  const bool boxed_in = near_count >= 3;

  if (boxed_in || ((pressed_forward || pressed_reverse) && little_change))
  {
    Serial.printf("Unstuck: dir=%c front=%u rear=%u rear_left=%u rear_right=%u left=%u right=%u\n",
                  direction,
                  static_cast<unsigned>(front_now),
                  static_cast<unsigned>(lidar_state.rear_min_mm),
                  static_cast<unsigned>(lidar_state.rear_left_min_mm),
                  static_cast<unsigned>(lidar_state.rear_right_min_mm),
                  static_cast<unsigned>(lidar_state.left_min_mm),
                  static_cast<unsigned>(lidar_state.right_min_mm));
    start_unstuck_escape();
    return true;
  }

  stuck_tracker.start_ms = millis();
  stuck_tracker.front_mm = front_now;
  stuck_tracker.rear_mm = lidar_state.rear_min_mm;
  stuck_tracker.left_mm = lidar_state.left_min_mm;
  stuck_tracker.right_mm = lidar_state.right_min_mm;
  return false;
}

void activate_mode(char new_mode)
{
  mode = new_mode;
  direction = 'x';
  g_dodging = false;
  g_unsticking = false;
  reset_wander_state();
  reset_stuck_tracker();
  stop_drive();
  Serial.printf("Mode -> %c\n", mode);
}

} // namespace bot
