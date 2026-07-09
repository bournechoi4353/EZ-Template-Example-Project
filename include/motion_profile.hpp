#pragma once

// trapezoidal profile for straight drives. standalone -- doesn't touch
// EZ-Template, just borrows the motors.
//   target_in = inches, negative = backward
//   max_vel = in/s, max_accel = in/s^2
// tuning constants (kV/kA/kP) live in the .cpp
void profiled_drive(double target_in, double max_vel = 30.0, double max_accel = 60.0);
