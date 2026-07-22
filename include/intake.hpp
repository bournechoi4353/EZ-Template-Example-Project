#pragma once

#include "api.h"

extern pros::Motor intake;

// Sets the intake speed (-127 to 127, positive = in) for use in autonomous.
void set_intake(int speed);

// Driver control: hold R1 = in, hold R2 = out, release = stop.
void intake_control();
