#include "main.h"
#include "field_map.hpp"

#include <cmath>

// see wall_reset.hpp for the approach. tuning + the sensor table live here.
namespace {

struct sensor_mount {
  pros::Distance* dev;
  double off_x, off_y;   // body frame: +x right, +y forward, inches
  double facing_deg;     // CW from the robot's front
};

// distance sensors are DISABLED for now (see ports.hpp) -- SENSORS is empty,
// wall_reset() below just returns false immediately. uncomment the table once
// the sensors are wired back in.
const sensor_mount SENSORS[] = {};
// const sensor_mount SENSORS[] = {
//     {&distanceFront, DIST_FRONT_OFF_X, DIST_FRONT_OFF_Y, DIST_FRONT_FACING},
//     {&distanceRight, DIST_RIGHT_OFF_X, DIST_RIGHT_OFF_Y, DIST_RIGHT_FACING},
//     {&distanceLeft,  DIST_LEFT_OFF_X,  DIST_LEFT_OFF_Y,  DIST_LEFT_FACING},
// };

const int    MIN_CONFIDENCE = 30;    // 0-63, same gate the rest of the project uses
const double MIN_DIST_IN    = 2.0;
const double MAX_DIST_IN    = 36.0;  // farther than this probably isn't a wall
const double ALIGN_TOL_DEG  = 7.0;   // how square we have to be to trust the reading
const double SANITY_MARGIN  = 3.0;   // reject a result this far outside the field

double wrap180(double a) {
  while (a >= 180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

// one sensor's vote for a coordinate. confidence is 0..1 (raw 0-63 / 63).
struct vote {
  bool is_x;
  double value;
  double confidence;
};

// TitanReset's core trick, ported to body-frame offsets: correct the raw
// reading for squareness error, then add the sensor's own mount offset
// (decomposed into components along/across its own beam) -- gives the
// perpendicular distance from robot CENTER to the wall the sensor faces.
bool sensor_vote(const sensor_mount& m, double heading_deg, vote& out) {
  if (m.dev->get_confidence() < MIN_CONFIDENCE) return false;
  double raw_in = m.dev->get() / 25.4;  // mm -> in
  if (raw_in < MIN_DIST_IN || raw_in > MAX_DIST_IN) return false;

  // which way the beam points in world space, and how far off a cardinal
  // (wall-facing) direction it is right now
  double world = std::fmod(heading_deg + m.facing_deg, 360.0);
  if (world < 0) world += 360.0;
  double nearest = std::round(world / 90.0) * 90.0;
  double err_deg = wrap180(world - nearest);
  if (std::fabs(err_deg) > ALIGN_TOL_DEG) return false;  // not square to a wall
  int card = static_cast<int>(std::round(nearest / 90.0)) % 4;  // 0:+Y 1:+X 2:-Y 3:-X

  // decompose the mount offset into components along ("parallel") and
  // across ("perp") the sensor's own beam -- fixed per sensor, since
  // facing_deg is body-relative and doesn't depend on heading
  double fr = ez::util::to_rad(m.facing_deg);
  double parallel = m.off_x * std::sin(fr) + m.off_y * std::cos(fr);
  double perp = m.off_x * std::cos(fr) - m.off_y * std::sin(fr);

  double err_rad = ez::util::to_rad(err_deg);
  double perp_dist = std::cos(err_rad) * raw_in
                    + std::cos(err_rad) * parallel
                    - std::sin(err_rad) * perp;

  double value;
  bool is_x;
  switch (card) {
    case 0: value = field::FIELD_HALF - perp_dist;  is_x = false; break;  // +Y wall
    case 1: value = field::FIELD_HALF - perp_dist;  is_x = true;  break;  // +X wall
    case 2: value = -field::FIELD_HALF + perp_dist; is_x = false; break;  // -Y wall
    default: value = -field::FIELD_HALF + perp_dist; is_x = true; break;  // -X wall
  }

  if (std::fabs(value) > field::FIELD_HALF + SANITY_MARGIN) return false;

  out = {is_x, value, m.dev->get_confidence() / 63.0};
  return true;
}

}  // namespace

bool wall_reset() {
  double heading = chassis.odom_theta_get();  // IMU-backed, never written here

  double x_sum = 0, x_wsum = 0;
  double y_sum = 0, y_wsum = 0;

  for (const sensor_mount& m : SENSORS) {
    vote v;
    if (!sensor_vote(m, heading, v)) continue;
    if (v.is_x) { x_sum += v.value * v.confidence; x_wsum += v.confidence; }
    else        { y_sum += v.value * v.confidence; y_wsum += v.confidence; }
  }

  bool applied = false;
  if (x_wsum > 0.0) { chassis.odom_x_set(x_sum / x_wsum); applied = true; }
  if (y_wsum > 0.0) { chassis.odom_y_set(y_sum / y_wsum); applied = true; }
  return applied;
}
