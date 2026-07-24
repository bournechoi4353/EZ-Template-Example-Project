#pragma once

#include "api.h"

// wall resets off the distance sensors. when we're squared up against a wall
// the sensor tells us exactly how far it is, so we can recompute the robot's
// real x or y and snap odom to it. only call these when the robot is actually
// facing/backed into a wall, mid-field the sensor just sees robots and
// game pieces.

// reset one axis from one sensor.
// mount_angle_deg = which way the sensor points, clockwise from the front
// (0 front, 90 right, 180 back, 270 left)
// offset_in = robot center to the sensor face (measure it)
// returns false if the reading got rejected (low confidence, out of range,
// not square to a wall, or a result outside the field)
bool wall_reset(pros::Distance& sensor, double mount_angle_deg, double offset_in);

// try all three sensors. back + a side can fix x and y in one call
void wall_reset_all();
