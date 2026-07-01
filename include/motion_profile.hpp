#pragma once

// Trapezoidal motion profile for a straight drive (a self-contained add-on --
// it does NOT modify EZ-Template, it just borrows the motors via drive_set()).
//
//   target_in  = distance to drive, in inches (negative = backward)
//   max_vel    = top speed of the profile, inches / second
//   max_accel  = how hard it ramps, inches / second^2
//
// See src/motion_profile.cpp for the tuning constants (kV / kA / kP).
void profiled_drive(double target_in, double max_vel = 30.0, double max_accel = 60.0);
