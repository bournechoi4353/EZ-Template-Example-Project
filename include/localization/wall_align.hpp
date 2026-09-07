#pragma once

#include "api.h"

// closed-loop drive + heading helpers that steer off the VEX distance sensors
// instead of the drive encoders. use these for the last few inches into a wall
// or goal, where encoder drift has already compounded but a sensor sees the
// real distance. mid-field the sensors just see robots / game pieces, so only
// call these when the beam is actually pointed at a wall (same rule as mcl).
//
// noise: VEX distance sensors are jumpy at range, so every helper averages a
// handful of samples and throws out low-confidence ones rather than trusting a
// single .get().

// drive straight forward/backward under a P loop until `sensor` reads
// target_mm. pass a FORWARD- or BACK-facing sensor (e.g. distanceFront): the
// loop assumes driving forward shrinks the reading, so reading > target drives
// forward and reading < target backs up. max_speed is the magnitude cap
// (0-127). does NOT use the horizontal tracker -- forward distance comes from
// the sensor, the tracker stays lateral-only.
void drive_to_distance(pros::Distance& sensor, int target_mm, int max_speed);

// turn in place until the left and right distance sensors read within
// tolerance_mm of each other. standalone correction step.
//
// GEOMETRY WARNING -- read before trusting this to fix heading:
//   matching left vs right squares HEADING only when both beams hit the SAME
//   wall (or two non-parallel walls). the left/right sensors on this bot face
//   OPPOSITE walls (right=90 deg, left=270 deg). if those two walls are
//   parallel (opposing perimeter walls are), left==right is heading-invariant
//   -- it locates the robot CENTERED between the walls, not squared. so on a
//   straight corridor this corrects lateral centering, not heading, and can
//   chase an unreachable match. it is bounded (small clamped nudges, capped
//   iterations, gated on valid wall readings) so a wrong-geometry call is a
//   no-op, never a runaway. keep the IMU as the heading source (see mcl.hpp).
void square_to_walls(int tolerance_mm);
