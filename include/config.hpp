#pragma once

// ============================================================
//  robot config, every port and measurement lives here.
//  changed a port on the robot? change it once, right here,
//  and everything else picks it up.
// ============================================================

//  drive: 2 full (11W) + 1 half (5.5W) motor per side (negative port = reversed) 
// each side is {full, full, half}. placeholders, set to your real ports and negate
// any motor that spins the wrong way.
#define LEFT_DRIVE_PORTS  {-15, -16, -14}      // {full, full, half}
#define RIGHT_DRIVE_PORTS {12, 13, 11}   // {full, full, half}
constexpr int    IMU_PORT       = 9;   // V5 inertial sensor
constexpr double WHEEL_DIAMETER = 2.75;
constexpr double DRIVE_RPM      = 450;

//  tracking wheels (offsets come from the Measure Offsets auton) 
// the horiz sign matters, +7.61 ran x/y away on a spin
constexpr int    HORIZ_TRACKER_PORT     = 6;
constexpr double HORIZ_TRACKER_DIAMETER = 3.25;
constexpr double HORIZ_TRACKER_OFFSET   = -7.61;

constexpr int    VERT_TRACKER_PORT      = -15;  // negative = reversed
constexpr double VERT_TRACKER_DIAMETER  = 2.75;
constexpr double VERT_TRACKER_OFFSET    = 1.05;

//  arm (placeholder ports, change to your real ports; negative = reversed) 
constexpr int ARM_LEFT_PORT  = 17;
constexpr int ARM_RIGHT_PORT = 18;

//  arm height presets (motor-encoder degrees) 
// PLACEHOLDERS. run the "Arm Height Test" auton, jog the arm to each real tower
// height, read the printed encoder value, and drop the real numbers in here.
constexpr double ARM_REST_POS = 0;
constexpr double ARM_LOW_POS  = 900;
constexpr double ARM_MID_POS  = 100;
constexpr double ARM_HIGH_POS = 2300;

//  intake (positive = in) 
constexpr int INTAKE_PORT = 19;

//  pneumatics (ADI ports 'A'-'H'), PLACEHOLDERS, set to your real ports 
constexpr char ROTATE_PORT = 'A';  // claw rotate
constexpr char GRAB_PORT    = 'B';  // claw grab

//  distance sensors (NONE installed yet, MCL reads these; keep them OFF
// the drive/motor ports. set real ports when you add sensors) 
constexpr int DISTANCE_BACK_PORT  = 8;
constexpr int DISTANCE_RIGHT_PORT = 7;   // moved off 9 (IMU is there now)
constexpr int DISTANCE_LEFT_PORT  = 10;