#pragma once

#include "api.h"

// =============================================================================
//  Distance-sensor WALL RESETS
//  ------------------------------------------------------------------------
//  When the robot is squared up against a known field wall, a distance sensor
//  tells us exactly how far that wall is -- so we can recompute the robot's true
//  X or Y and snap odom to it, erasing accumulated drift on that axis.
//
//  Call these only when you KNOW the robot is roughly facing / backed into a
//  wall (e.g. right after driving into it to align) -- NOT mid-field, where the
//  sensor would see robots or game objects.
// =============================================================================

// Reset one axis from one sensor.
//   mount_angle_deg = the direction the sensor points, measured CLOCKWISE from
//                     the robot's FRONT (0 = front, 90 = right, 180 = back,
//                     270 = left).
//   offset_in       = distance from robot CENTER to the sensor's face, along
//                     that pointing direction (~half the robot, MEASURE it).
// Returns true if a reset happened, false if the reading was rejected (low
// confidence, out of range, not square to a wall, or an implausible result).
bool wall_reset(pros::Distance& sensor, double mount_angle_deg, double offset_in);

// Try all three mounted sensors (back / left / right).  Each valid one resets
// its axis, so a back + side combo can fix X and Y in one call.
void wall_reset_all();
