#pragma once

// all the PID/odom tuning, gets called in initialize()
void default_constants();

// spins the robot a bunch and calculates the tracking wheel offsets
void measure_offsets();

// tight IMU 90s, relative to wherever we're facing now
void turn_right_90(int speed = 90);
void turn_left_90(int speed = 90);

// turn 90 and print how far off we landed
void turn_test();

// push the robot by hand, check the trackers count the right way
void tracker_dir_test();

// motor pivot in place -- x/y drift should stay ~0
void odom_spin_test();

// drive straight forward 24" with odom, should end ~(0,24,0)
void odom_drive_test();

// drift demo + the two autons built on it
void drift_demo(int high, int low, int sweepMs);
void BC2145AUTO();
void BC2145AUTO_fullsend();

void pistons();

// run a path.jerryio export through pure pursuit
void jerryio_path_example();

// main match auton
void overrideTest();

// odom playground -- drive a square, see the drift
void odom_sandbox();

// motion profiling demos
void motion_profile_test();
void profiling_showcase();

void wall_reset_test();

// MCL test: catches a 4" odom error we plant on purpose
void mcl_test();
