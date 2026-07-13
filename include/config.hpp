#pragma once

// ============================================================
//  robot config -- every port and measurement lives here.
//  changed a port on the robot? change it once, right here,
//  and everything else picks it up.
// ============================================================

// ---- drive (negative port = reversed) ----
#define LEFT_DRIVE_PORTS  {-3, 4, -5}
#define RIGHT_DRIVE_PORTS {11, 12, -13}
constexpr int    IMU_PORT       = 21;
constexpr double WHEEL_DIAMETER = 3.25;
constexpr double DRIVE_RPM      = 450;

// ---- tracking wheels (offsets come from the Measure Offsets auton) ----
// the horiz sign matters -- +7.61 ran x/y away on a spin
constexpr int    HORIZ_TRACKER_PORT     = 6;
constexpr double HORIZ_TRACKER_DIAMETER = 3.25;
constexpr double HORIZ_TRACKER_OFFSET   = -7.61;

constexpr int    VERT_TRACKER_PORT      = -15;  // negative = reversed
constexpr double VERT_TRACKER_DIAMETER  = 2.75;
constexpr double VERT_TRACKER_OFFSET    = 1.05;

// ---- intake motors (negative = reversed, both blue = 600rpm) ----
constexpr int BOTTOM_INTAKE_PORT = -16;
constexpr int TOP_INTAKE_PORT    = -17;

// ---- pistons (ADI ports, A is free) ----
constexpr char LOADER_PORT   = 'B';
constexpr char INDEXER_PORT  = 'C';
constexpr char WING_PORT     = 'D';
constexpr char HOOD_PORT     = 'E';
constexpr char ANTIDUMP_PORT = 'F';
constexpr char LIFT_PORT     = 'G';
constexpr char DESCORE_PORT  = 'H';

// ---- distance sensors ----
constexpr int DISTANCE_BACK_PORT  = 8;
constexpr int DISTANCE_RIGHT_PORT = 18;
constexpr int DISTANCE_LEFT_PORT  = 19;
