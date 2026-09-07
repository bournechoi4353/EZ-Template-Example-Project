#pragma once

//  drivetrain: 2 full (11W) + 1 half (5.5W) per side, {front, middle, back}.
//  LEFT joystick drives the physically-RIGHT motors and vice versa (flipped
//  on purpose). sign is a DEFAULT, not measured -- flip a side if it drives
//  backward.
#define LEFT_DRIVE_PORTS  {-18, 19, -20}    // physically front-right/mid-right/back-right
#define RIGHT_DRIVE_PORTS {12, -11, 5}  // physically front-left/mid-left/back-left
constexpr int    IMU_PORT       = 13;   // gyro
constexpr double WHEEL_DIAMETER = 2.75;
constexpr double DRIVE_RPM      = 450;

//  lift: single motor
constexpr int LIFT_PORT = -7;
constexpr int LIFT2_PORT = 9;

//  arm rotation sensor (absolute position feedback for the lift)
constexpr int ARM_SENSOR_PORT = 10;

//  arm travel limits, in rotation-sensor degrees. hard stop -- the arm is
//  never allowed outside this range, driving or auton.
constexpr double ARM_SENSOR_LOWER_LIMIT = -20;
constexpr double ARM_SENSOR_UPPER_LIMIT = 94;

constexpr double ARM_SPEED_LIMIT_THRESHOLD = 45;
constexpr int ARM_SPEED_LIMIT = 90;


//  arm/lift height presets (motor-encoder degrees), captured via "Arm Height
//  Capture". PLACE = height to actually set the pin down, HOVER = a bit
//  higher, for approaching/clearing before placing.
constexpr double ARM_REST_POS = 0;

constexpr double ARM_PIN1_PLACE_POS = 333;
constexpr double ARM_PIN1_HOVER_POS = 500;

constexpr double ARM_PIN2_PLACE_POS = 800;
constexpr double ARM_PIN2_HOVER_POS = 950;

constexpr double ARM_PIN3_PLACE_POS = 1220;
constexpr double ARM_PIN3_HOVER_POS = 1400;

constexpr double ARM_PIN4_PLACE_POS = 1500;
constexpr double ARM_PIN4_HOVER_POS = 1900;

//  claw roller: 2x 5.5W motors as one group (negative port = reversed).
//  PLACEHOLDERS -- swap these for the real ports once it's wired.
constexpr int ROLLER_RIGHT_PORT = -8;
constexpr int ROLLER_LEFT_PORT  = 3;

//  claw clamp, the robot's only pneumatic (ADI port 'A'-'H'). R1 opens it,
//  L1 closes it (see controls.cpp).
constexpr char CLAW_PISTON_PORT = 'A';

//  distance sensors -- DISABLED for now, commented out to avoid any issues
//  (port conflicts, stale sensor roles, etc). uncomment when actually wiring
//  these up. mcl.cpp/wall_reset.cpp/wall_align.cpp/skills() all had their
//  sensor-dependent parts safely no-op'd to match.
// constexpr int DISTANCE_FRONT_PORT = 15;
// constexpr int DISTANCE_LEFT_PORT  = 17;
// constexpr int DISTANCE_RIGHT_PORT = 2;

// constexpr double DIST_FRONT_OFF_X = -5.0, DIST_FRONT_OFF_Y = 7.0, DIST_FRONT_FACING = 0.0;
// constexpr double DIST_RIGHT_OFF_X = 4.5,  DIST_RIGHT_OFF_Y = 1.5, DIST_RIGHT_FACING = 90.0;
// constexpr double DIST_LEFT_OFF_X  = -5.0, DIST_LEFT_OFF_Y  = 6.5, DIST_LEFT_FACING  = 270.0;
