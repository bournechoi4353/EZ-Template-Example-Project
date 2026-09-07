#include "main.h"

#include <cmath>

// see wall_align.hpp for the geometry caveats. tuning lives here.

namespace {
const int    SAMPLES        = 5;      // reads averaged per measurement
const int    MIN_CONFIDENCE = 30;     // 0-63, same gate mcl uses
const int    MAX_VALID_MM   = 2000;   // farther than this probably isn't a wall

// drive_to_distance
const double DRIVE_KP    = 0.5;       // mm error -> motor power
const int    DRIVE_MIN   = 8;         // floor so it still creeps when close
const int    DRIVE_TOL   = 15;        // mm; inside this counts as "there"
const int    DRIVE_SETTLE = 3;        // consecutive in-tol reads before stopping
const int    DRIVE_TIMEOUT_MS = 3000; // safety, never stall forever

// square_to_walls
const int    SQ_TURN_SPEED   = 90;    // out of 127
const int    SQ_MAX_ITERS    = 6;
const double SQ_DEG_PER_MM   = 0.05;  // diff (mm) -> turn (deg), intentionally gentle
const double SQ_PER_ITER_DEG = 5.0;   // clamp per correction
const double SQ_MAX_TOTAL_DEG = 12.0; // clamp total, so a wrong sign can't run away

// average SAMPLES reads, dropping any that are low-confidence or out of wall
// range. returns -1 if nothing was trustworthy this call.
double avg_distance_mm(pros::Distance& sensor) {
  double sum = 0.0;
  int used = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int r = sensor.get();
    if (sensor.get_confidence() >= MIN_CONFIDENCE && r > 0 && r < MAX_VALID_MM) {
      sum += r;
      used++;
    }
    pros::delay(ez::util::DELAY_TIME);
  }
  return used > 0 ? sum / used : -1.0;
}

int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

void drive_to_distance(pros::Distance& sensor, int target_mm, int max_speed) {
  max_speed = std::abs(max_speed);
  int settled = 0;
  uint32_t start = pros::millis();

  while (pros::millis() - start < (uint32_t)DRIVE_TIMEOUT_MS) {
    double reading = avg_distance_mm(sensor);

    // no trustworthy read -> coast, don't drive blind. timeout still applies.
    if (reading < 0) {
      chassis.drive_set(0, 0);
      continue;
    }

    double err = reading - target_mm;  // + => too far => drive forward
    if (std::fabs(err) <= DRIVE_TOL) {
      if (++settled >= DRIVE_SETTLE) break;
    } else {
      settled = 0;
    }

    int power = (int)std::round(DRIVE_KP * err);
    power = clampi(power, -max_speed, max_speed);
    // keep a minimum push once we're outside tolerance so friction doesn't stall us
    if (std::abs(power) < DRIVE_MIN && std::fabs(err) > DRIVE_TOL)
      power = err > 0 ? DRIVE_MIN : -DRIVE_MIN;

    chassis.drive_set(power, power);
    pros::delay(ez::util::DELAY_TIME);
  }

  chassis.drive_set(0, 0);
}

// distance sensors are DISABLED for now (see ports.hpp) -- this is a safe
// no-op until they're wired back in (uncomment the body below).
void square_to_walls(int tolerance_mm) {
  return;

  /*
  double applied = 0.0;

  for (int i = 0; i < SQ_MAX_ITERS; i++) {
    double l = avg_distance_mm(distanceLeft);
    double r = avg_distance_mm(distanceRight);

    // one side isn't seeing a wall -> can't square off junk, bail cleanly.
    if (l < 0 || r < 0) return;

    double diff = l - r;
    if (std::fabs(diff) <= tolerance_mm) return;  // squared enough

    // sign convention may need flipping for your field; it's clamped so a wrong
    // sign just no-ops inside the total cap rather than walking heading away.
    double turn = clampd(diff * SQ_DEG_PER_MM, -SQ_PER_ITER_DEG, SQ_PER_ITER_DEG);
    turn = clampd(turn, -SQ_MAX_TOTAL_DEG - applied, SQ_MAX_TOTAL_DEG - applied);
    if (std::fabs(turn) < 1e-3) return;  // hit the total clamp, stop

    chassis.pid_turn_relative_set(turn, SQ_TURN_SPEED);
    chassis.pid_wait();
    applied += turn;
  }
  */
}
