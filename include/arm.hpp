#pragma once

#include "api.h"

extern pros::MotorGroup arm;
extern pros::Rotation arm_sensor;

// arm speed for autons, -127..127 (positive = up)
void set_arm(int speed);
