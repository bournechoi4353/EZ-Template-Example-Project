#pragma once

// all the PID/odom tuning, gets called in initialize()
void default_constants();

// cycles the arm through the preset heights so you can check/capture them
void arm_height_test();

// simple match auton: drive + turn via EZ's IMU PID
void overrideTest();

// programming skills run (60s solo)
void skills();

// motor pivot in place -- x/y drift should stay ~0
void odom_spin_test();

// 48in square (straights + IMU turns) -- tune drive/turn PID by how well it closes
void pid_square();

// straight 48" via the trapezoid motion profile, prints distance traveled
void motion_profile_test();

// MCL test: catches a 4" odom error we plant on purpose
void mcl_test();
