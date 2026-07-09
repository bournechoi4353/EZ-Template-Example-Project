#include "main.h"

#include <cmath>

namespace {
const double FIELD_HALF = 70.25;   // center to inner wall face
const int MIN_CONFIDENCE = 30;     // sensor confidence is 0-63
const double MIN_DIST_IN = 2.0;    // closer than this is unreliable
const double MAX_DIST_IN = 36.0;   // farther than this probably isn't a wall
const double ALIGN_TOL_DEG = 7.0;  // how square we have to be
const double SANITY_MARGIN = 3.0;  // reject results this far outside the field

double wrap180(double a) {
  while (a >= 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
}  // namespace

bool wall_reset(pros::Distance& sensor, double mount_angle_deg, double offset_in) {
  // can we trust the reading at all?
  if (sensor.get_confidence() < MIN_CONFIDENCE) return false;
  double reading_in = sensor.get() / 25.4;  // mm -> in
  if (reading_in < MIN_DIST_IN || reading_in > MAX_DIST_IN) return false;

  // which way is the beam pointing, and are we square to a wall?
  double beam = chassis.odom_theta_get() + mount_angle_deg;
  beam = std::fmod(beam, 360.0);
  if (beam < 0) beam += 360.0;

  double nearest = std::round(beam / 90.0) * 90.0;
  double err = wrap180(beam - nearest);
  if (std::fabs(err) > ALIGN_TOL_DEG) return false;
  int card = static_cast<int>(std::round(nearest / 90.0)) % 4;  // 0..3

  // perpendicular distance from robot center to the wall (the cos cleans up
  // the small misalignment)
  double perp = (reading_in + offset_in) * std::cos(ez::util::to_rad(err));

  // snap whichever axis faces that wall. card: 0=+Y, 1=+X, 2=-Y, 3=-X
  double value;
  bool is_x;
  switch (card) {
    case 0: value = FIELD_HALF - perp;  is_x = false; break;
    case 1: value = FIELD_HALF - perp;  is_x = true;  break;
    case 2: value = -FIELD_HALF + perp; is_x = false; break;
    default: value = -FIELD_HALF + perp; is_x = true; break;
  }

  // a real on-field reset has to land inside the field
  if (std::fabs(value) > FIELD_HALF + SANITY_MARGIN) return false;

  if (is_x)
    chassis.odom_x_set(value);
  else
    chassis.odom_y_set(value);
  return true;
}

void wall_reset_all() {
  // mount angle (front=0, right=90, back=180, left=270) + center-to-face
  // offset. the offsets are still placeholders, measure them
  wall_reset(distanceBack, 180.0, 6.3);
  wall_reset(distanceRight, 90.0, 6.3);
  wall_reset(distanceLeft, 270.0, 6.3);
}
