#pragma once

// one-shot odom snap off the front/right/left distance sensors -- whichever
// of them is currently square to a wall votes for the x and/or y coordinate,
// weighted by its confidence, and odom gets set directly. no particle filter,
// no waiting for a cloud to converge -- just "read the walls right now."
//
// based on team 38535A's TitanReset (github.com/tubaplayerdis/TitanReset):
// same core trick -- correct the raw reading for how "un-square" the robot
// is to the wall, then add the sensor's own mount offset projected onto its
// own beam -- adapted here because this robot has no back sensor (TitanReset
// assumes a full front/right/back/left ring) and, unlike TitanReset, this
// NEVER sets heading. Only x/y get written, same rule mcl.hpp follows and for
// the same reason: this field is 4-fold symmetric, so nothing but the IMU
// gets to touch heading.
//
// call this between motions (after a pid_wait()), not while translating --
// same rule as mcl::flush_if_safe() and wall_align's helpers.
//
// returns true if at least one axis was confidently updated.
bool wall_reset();
