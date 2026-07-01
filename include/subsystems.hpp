#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');

// Pneumatic pistons (ADI ports).  Port A is unused.
//   .extend() fires it, .retract() pulls it back, .toggle() flips it.
inline pros::adi::Pneumatics loader('B', false);    // starts retracted
inline pros::adi::Pneumatics indexer('C', false);
inline pros::adi::Pneumatics wing('D', false);
inline pros::adi::Pneumatics hood('E', false);
inline pros::adi::Pneumatics antiDump('F', false);
inline pros::adi::Pneumatics lift('G', false);
inline pros::adi::Pneumatics descore('H', true);  // ALWAYS up -- starts extended at boot

// Distance sensors (smart ports).  .get() returns millimeters.
inline pros::Distance distanceBack(8);
inline pros::Distance distanceRight(18);
inline pros::Distance distanceLeft(19);
// front sensor port is unconfirmed -- 0 isn't a valid smart port (1-21),
// so update this to the real port before uncommenting.
// inline pros::Distance distanceFront(0);