#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "config.hpp"  // ports live here

extern Drive chassis;

// pistons. extend/retract/toggle
inline pros::adi::Pneumatics loader(LOADER_PORT, false);
inline pros::adi::Pneumatics indexer(INDEXER_PORT, false);
inline pros::adi::Pneumatics wing(WING_PORT, false);
inline pros::adi::Pneumatics hood(HOOD_PORT, false);
inline pros::adi::Pneumatics antiDump(ANTIDUMP_PORT, false);
inline pros::adi::Pneumatics lift(LIFT_PORT, false);
inline pros::adi::Pneumatics descore(DESCORE_PORT, true);  // always up, starts extended at boot

// distance sensors, .get() returns mm
inline pros::Distance distanceBack(DISTANCE_BACK_PORT);
inline pros::Distance distanceRight(DISTANCE_RIGHT_PORT);
inline pros::Distance distanceLeft(DISTANCE_LEFT_PORT);
// no front sensor yet -- add the port to config.hpp before uncommenting
// inline pros::Distance distanceFront(...);
