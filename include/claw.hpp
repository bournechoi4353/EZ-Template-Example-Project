#pragma once

#include "api.h"

// roller motor group (2x 5.5W)
extern pros::MotorGroup roller;
extern pros::Motor rotationClaw;

// claw clamp piston: open/close (see controls.cpp: R1 open, L1 close)
extern pros::adi::Pneumatics clawPiston;

// roller speed for autons, -127..127 (positive = in)
void set_roller(int speed);
void rotateClaw();

