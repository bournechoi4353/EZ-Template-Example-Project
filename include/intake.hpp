#pragma once

#include "api.h"

extern pros::Motor bottom_intake_left;
extern pros::Motor top_intake_right;

// Sets the bottom intake speed (-127 to 127) for use in autonomous.
void set_bottom_intake(int speed);

// Driver control: R1 = intake in, R2 = reverse (both motors); L1 hold = hood stopper down.
void intake_control();
