#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "config.hpp"  // ports live here

extern Drive chassis;

// pneumatics. extend() / retract() / toggle()
inline pros::adi::Pneumatics rotate(ROTATE_PORT, false);  // claw rotate
inline pros::adi::Pneumatics grab(GRAB_PORT, false);      // claw grab

// distance sensors, .get() returns mm
inline pros::Distance distanceBack(DISTANCE_BACK_PORT);
inline pros::Distance distanceRight(DISTANCE_RIGHT_PORT);
inline pros::Distance distanceLeft(DISTANCE_LEFT_PORT);
// no front sensor yet -- add the port to config.hpp before uncommenting
// inline pros::Distance distanceFront(...);
