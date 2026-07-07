#pragma once

#include <cmath>

/**
 * Static field map for the 2026-27 game "Override" -- the geometry MCL
 * ray-casts against.
 *
 * v1 = PERIMETER WALLS ONLY.  Walls are the most reliable static geometry on
 * this field (56 cups + 63 pins + robots all move), and they're the only
 * numbers we have fully confirmed.  Goal + loader bodies get added in v2 once
 * Appendix A confirms their real dimensions -- their nominal centers are
 * already listed below so v2 is just "add circle/segment intersection".
 *
 * Frame (matches EZ odom + path.jerryio): origin = field center, +X = right,
 * +Y = forward, heading 0 = +Y, CLOCKWISE positive.  ONE trig rule everywhere:
 * a heading h maps to direction (sin h, cos h).  No raw CCW atan2 anywhere --
 * that's how the mirror bug stays impossible.
 */
namespace field {

// Center -> INNER face of the field wall (same number wall_reset uses).
constexpr double FIELD_HALF = 70.25;

// Nominal foam-tile pitch.  [CONFIRM Appendix A: 24.0 vs ~23.4 -- goal centers
// below shift with it, up to ~2.4"]
constexpr double TILE = 24.0;

// --- v2 geometry (NOT ray-cast yet; here so the data lives with the map) ---
// 9 goals, modeled as circular range targets at beam height (<~4").
// Diameter: [FILL FROM APPENDIX A -- use the cross-section AT BEAM HEIGHT,
// the base can differ from the upper structure].
constexpr double GOAL_CENTERS[9][2] = {
    {0, 0},  // center goal
    {+TILE, +2 * TILE}, {+TILE, -2 * TILE}, {-TILE, +2 * TILE}, {-TILE, -2 * TILE},
    {+2 * TILE, +TILE}, {+2 * TILE, -TILE}, {-2 * TILE, +TILE}, {-2 * TILE, -TILE},
};
// Loaders: 4, in the corners, inset toward center from (+/-FIELD_HALF, +/-FIELD_HALF).
// Need center + size + shape at beam height before they can be mapped.
// Toggles (wall centers): permanent OUTLIER zones -- they rotate, never mapped.

/**
 * Distance from (x, y) along unit direction (dx, dy) to the field wall.
 *
 * v1: ray-vs-AABB "slab" exit distance -- from inside the wall square the ray
 * exits through exactly one wall, and that exit distance IS the predicted
 * range.  Returns -1 if (x, y) is outside the walls (a particle that drifted
 * out of the field -- caller weights it with the outlier floor and it dies in
 * resampling).
 *
 * v2 slots in here: predicted = min(wall exit, nearest goal circle hit,
 * nearest loader hit).  Callers never change.
 */
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

  return (t > 1e17) ? -1.0 : t;  // 1e17: degenerate zero direction
}

}  // namespace field
