#pragma once

#include "api.h"

extern pros::MotorGroup arm;

// Sets the arm speed (-127 to 127, positive = up) for use in autonomous.
void set_arm(int speed);

// Driver control: hold L1 = arm up, hold L2 = arm down, release = hold position.
void arm_control();
