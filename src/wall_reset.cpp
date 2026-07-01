#include "main.h"

#include <cmath>

// ---- Tunables ----
namespace {
const double FIELD_HALF = 70.25;   // center to inner wall face (from your robot's known-good config)
const int MIN_CONFIDENCE = 30;     // skip if sensor confidence < this (0-63)
const double MIN_DIST_IN = 2.0;    // reading smaller than this is unreliable
const double MAX_DIST_IN = 36.0;   // reading bigger than this probably isn't a wall
const double ALIGN_TOL_DEG = 7.0;  // must be within this many degrees of square
const double SANITY_MARGIN = 3.0;  // reject results this far outside the field

// Wrap an angle to [-180, 180).
double wrap180(double a) {
  while (a >= 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}
}  // namespace

bool wall_reset(pros::Distance& sensor, double mount_angle_deg, double offset_in) {
  // 1. Is the raw reading trustworthy?
  if (sensor.get_confidence() < MIN_CONFIDENCE) return false;
  double reading_in = sensor.get() / 25.4;  // mm -> inches
  if (reading_in < MIN_DIST_IN || reading_in > MAX_DIST_IN) return false;

  // 2. Which way is the beam pointing on the field, and is it square to a wall?
  double beam = chassis.odom_theta_get() + mount_angle_deg;
  beam = std::fmod(beam, 360.0);
  if (beam < 0) beam += 360.0;

  double nearest = std::round(beam / 90.0) * 90.0;  // nearest of 0/90/180/270/360
  double err = wrap180(beam - nearest);             // how far off square we are
  if (std::fabs(err) > ALIGN_TOL_DEG) return false;  // not facing a wall squarely
  int card = static_cast<int>(std::round(nearest / 90.0)) % 4;  // 0..3

  // 3. Perpendicular distance from robot CENTER to the wall (cos corrects the
  //    small misalignment).
  double perp = (reading_in + offset_in) * std::cos(ez::util::to_rad(err));

  // 4. Snap the axis facing that wall.  card: 0=+Y, 1=+X, 2=-Y, 3=-X.
  double value;
  bool is_x;
  switch (card) {
    case 0: value = FIELD_HALF - perp;  is_x = false; break;  // +Y wall
    case 1: value = FIELD_HALF - perp;  is_x = true;  break;  // +X wall
    case 2: value = -FIELD_HALF + perp; is_x = false; break;  // -Y wall
    default: value = -FIELD_HALF + perp; is_x = true; break;  // -X wall
  }

  // 5. Sanity: a real on-field reset must land inside the field.
  if (std::fabs(value) > FIELD_HALF + SANITY_MARGIN) return false;

  if (is_x)
    chassis.odom_x_set(value);
  else
    chassis.odom_y_set(value);
  return true;
}

void wall_reset_all() {
  // mount angle (front=0, right=90, back=180, left=270) + center-to-face offset.
  // The offsets are placeholders -- MEASURE them on your robot.
  wall_reset(distanceBack, 180.0, 6.3);   // points out the back
  wall_reset(distanceRight, 90.0, 6.3);   // points out the right
  wall_reset(distanceLeft, 270.0, 6.3);   // points out the left
}
