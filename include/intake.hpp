#pragma once

#include "api.h"

extern pros::Motor intake;

// intake speed for autons, -127..127 (positive = in)
void set_intake(int speed);

// driver control, hold R1 to intake, R2 to spit out, let go to stop
void intake_control();
