#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// pistons (ADI ports, A is free). extend/retract/toggle
inline pros::adi::Pneumatics loader('B', false);
inline pros::adi::Pneumatics indexer('C', false);
inline pros::adi::Pneumatics wing('D', false);
inline pros::adi::Pneumatics hood('E', false);
inline pros::adi::Pneumatics antiDump('F', false);
inline pros::adi::Pneumatics lift('G', false);
inline pros::adi::Pneumatics descore('H', true);  // always up, starts extended at boot

// distance sensors, .get() returns mm
inline pros::Distance distanceBack(8);
inline pros::Distance distanceRight(18);
inline pros::Distance distanceLeft(19);
// no front sensor yet -- put in the real port before uncommenting
// inline pros::Distance distanceFront(0);
