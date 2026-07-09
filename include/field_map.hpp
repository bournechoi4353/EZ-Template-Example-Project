#pragma once

#include <cmath>

// field geometry that MCL raycasts against. v1 is perimeter walls only --
// they're the only static thing on the field we've actually confirmed (cups,
// pins and robots all move). goal + loader shapes get added in v2 once we
// have real dimensions; their centers are staged below so v2 is mostly just
// adding circle intersection.
//
// frame matches EZ odom and path.jerryio: origin at field center, +x right,
// +y forward, heading 0 = +y, clockwise positive. one trig rule everywhere:
// heading h -> direction (sin h, cos h). no raw atan2 anywhere, that's how
// the mirror bug stays impossible.
namespace field {

// center to the inner wall face (same number wall_reset uses)
constexpr double FIELD_HALF = 70.25;

// nominal tile pitch. still need to confirm 24.0 vs ~23.4 -- the goal
// centers below shift with it, up to ~2.4"
constexpr double TILE = 24.0;

// v2 geometry, not raycast yet. 9 goals as circles at beam height.
// diameter TBD -- use the cross-section AT BEAM HEIGHT, not the base
constexpr double GOAL_CENTERS[9][2] = {
    {0, 0},  // center goal
    {+TILE, +2 * TILE}, {+TILE, -2 * TILE}, {-TILE, +2 * TILE}, {-TILE, -2 * TILE},
    {+2 * TILE, +TILE}, {+2 * TILE, -TILE}, {-2 * TILE, +TILE}, {-2 * TILE, -TILE},
};
// loaders sit in the 4 corners, need size/shape at beam height first.
// the wall toggles rotate so they're permanent outlier zones, never mapped.

// distance from (x, y) along unit direction (dx, dy) to the field wall.
// simple ray-vs-box exit distance: from inside the square the ray leaves
// through exactly one wall, and that's the predicted range. returns -1 if
// (x, y) is outside the walls -- that particle gets the outlier floor and
// dies in resampling. v2 becomes min(wall, goals, loaders) right here,
// callers never change.
inline double raycast(double x, double y, double dx, double dy) {
  if (std::fabs(x) > FIELD_HALF || std::fabs(y) > FIELD_HALF) return -1.0;

  constexpr double EPS = 1e-9;
  double t = 1e18;
  if (dx > EPS)
    t = (FIELD_HALF - x) / dx;
  else if (dx < -EPS)
    t = (-FIELD_HALF - x) / dx;
  if (dy > EPS)
    t = std::fmin(t, (FIELD_HALF - y) / dy);
  else if (dy < -EPS)
    t = std::fmin(t, (-FIELD_HALF - y) / dy);

  return (t > 1e17) ? -1.0 : t;  // 1e17 = degenerate zero direction
}

}  // namespace field
