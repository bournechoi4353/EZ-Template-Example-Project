#pragma once

// Drive constants / tuning (required -- called in initialize()).
void default_constants();

// Tracking-wheel offset calibration utility.
void measure_offsets();

// Precise IMU-based 90-degree turns (relative to current heading).
void turn_right_90(int speed = 90);
void turn_left_90(int speed = 90);

// Turns right 90 and reports the actual IMU heading + error (tuning check).
void turn_test();

// Live tracking-wheel direction check (push robot: forward=vert up, right=horiz up).
void tracker_dir_test();

// Motor-driven in-place pivot; reports odom x/y drift (should be ~0). Offset check.
void odom_spin_test();

// Simplest odom move: drive straight forward 24" from origin; should end ~(0,24,0).
void odom_drive_test();

// Drift demo + the two autons that call it (asymmetric-power sliding).
void drift_demo(int high, int low, int sweepMs);
void BC2145AUTO();
void BC2145AUTO_fullsend();

// Fires every piston in sequence, then retracts.
void pistons();

// Run a path.jerryio export via EZ-Template pure pursuit.
void jerryio_path_example();

// Match drive routine: intake + drives/turns to score, hood, reposition, wing.
void pushbacktest();

// Drive to field coordinates and print the final pose (odom playground).
void odom_sandbox();

// Trapezoidal motion-profiling demos.
void motion_profile_test();
void profiling_showcase();
void wall_reset_test();

// Monte Carlo localization test: converge near a corner, catch a deliberate
// 4" odom error, flush corrections between motions.
void mcl_test();
