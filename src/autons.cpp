#include "main.h"
#include "mcl_checkpoint.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

// milliseconds to wait between auton movements. change this one number; 0 = none
const int PAUSE = 0;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(12.0, 0.0, 80.0);          // Fwd/rev; lower kP = longer, gentler deceleration into the target (cruise speed unchanged)
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(4.0, 0.0, 52.5);    // Angular control for odom motions (kP lowered to stop the side-to-side sway)
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(8_in);           // How far ahead the robot looks -- smaller = less endpoint overshoot, bigger = smoother
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// simple match auton -- now uses EZ's IMU-based PID drive + turns (IMU on port 9).
void overrideTest() {

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  arm.move_absolute(ARM_LOW_POS, 70);   

  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);  
  chassis.pid_wait();

  
  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  arm.move_absolute(ARM_REST_POS, 100);
  pros::delay(1000);  

  chassis.pid_drive_set(-17_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-90_deg, TURN_SPEED); 
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  arm.move_absolute(ARM_LOW_POS, 100); 
  chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  arm.move_absolute(ARM_REST_POS, 100);
  pros::delay(1000);  

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();


  chassis.pid_drive_set(23_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

// programming skills run (60s solo). starter routine -- swap these blocks for
// your real field path. building blocks: pid_drive_set / pid_turn_set /
// pid_turn_relative_set + pid_wait, set_intake(speed), arm.move_absolute(pos, vel).
void skills() {

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  
  // repeat this figure 7 times. ABSOLUTE turns (not relative) so heading error
  // can't pile up over the reps -- every turn re-squares to a fixed heading.
  // assumes we enter the loop facing 90 (from the turn_set(90) above).
  for (int i = 0; i < 7; i++) {
    chassis.pid_wait_until(3000);
    chassis.pid_drive_set(-13_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_turn_set(-60_deg, TURN_SPEED);   // 90 - 150
    chassis.pid_wait();

    chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_wait_until(3000); 

    chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_turn_set(90_deg, TURN_SPEED);    // back to 90
    chassis.pid_wait();

    chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
    chassis.pid_wait();


  }
  chassis.pid_drive_set(-60_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(60_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  for (int i = 0; i < 7; i++) {
    chassis.pid_wait_until(3000);
    chassis.pid_drive_set(-13_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_turn_set(60_deg, TURN_SPEED);   // 90 - 150
    chassis.pid_wait();

    chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_wait_until(3000); 

    chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_turn_set(-90_deg, TURN_SPEED);    // back to 90
    chassis.pid_wait();

    chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
    chassis.pid_wait();


  }
}




// cycles the arm through each preset height, pausing at each so you can line it
// up against the tower and read the encoder. uses move_absolute -- the motor's
// built-in smooth move, no PID tuning. edit the ARM_*_POS numbers in config.hpp
// once you've captured your real heights.
void arm_height_test() {
  arm.tare_position_all();  // 0 = wherever the arm is resting right now

  const int ARM_VEL = 100;  // move speed in deg/sec (green cartridge tops out ~200)
  double heights[] = {ARM_LOW_POS, ARM_MID_POS, ARM_HIGH_POS, ARM_REST_POS};

  for (double h : heights) {
    arm.move_absolute(h, ARM_VEL);
    pros::delay(1500);  // let it get there and settle before reading

    printf("Arm -> target %.0f   actual %.1f\n", h, arm.get_position());
    ez::screen_print("Arm height test"
                         "\ntarget: " + util::to_string_with_precision(h) +
                         "\nactual: " + util::to_string_with_precision(arm.get_position()),
                     1);
  }
}

// pivot in place under motor power and report how far x/y drifted (should
// be ~0). don't spin it by hand -- your hands slide the robot and fake a
// drift. small drift = nudge the horiz offset, big runaway = wrong sign.
void odom_spin_test() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.drive_set(45, -45);   // left fwd, right rev = pivot
  pros::delay(4000);            // a few rotations
  chassis.drive_set(0, 0);
  pros::delay(400);

  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();
  double t = chassis.odom_theta_get();
  printf("Spin drift -> x: %.2f  y: %.2f  t: %.2f  (x/y should be ~0)\n", x, y, t);
  while (true) {
    ez::screen_print("After in-place pivot (x/y should be ~0):"
                         "\n x: " + util::to_string_with_precision(x) +
                         "\n y: " + util::to_string_with_precision(y) +
                         "\n t: " + util::to_string_with_precision(t),
                     1);
    pros::delay(ez::util::DELAY_TIME);
  }
}

// . . .
// Make your own autonomous functions here!
// . . .


// 48" square for tuning drive + turn PID -- four straights with 90 turns, no
// tracking wheels needed (IMU + encoders). watch how cleanly it closes back to
// the start: sides bowing = drive PID, corners off = turn PID.
void pid_square() {
  for (int i = 0; i < 4; i++) {
    chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    chassis.pid_turn_relative_set(90_deg, TURN_SPEED);  // 90 right; 4 corners = full loop
    chassis.pid_wait();
  }
}

// straight 48" with the trapezoid profile instead of PID, then prints how
// far it actually went. tune kV/kA/kP in motion_profile.cpp
void motion_profile_test() {
  // profiled_drive(inches, max vel in/s, max accel in/s^2)
  profiled_drive(48, 30, 60);

  double traveled = (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;
  printf("Motion profile -> target: 48.0 in   traveled: %f in\n", traveled);
}

// MCL end-to-end test. place the robot at field (-48, -48) facing forward,
// bottom-left corner: back wall ~22" behind center, left wall ~22" to the
// left, so both sensors are in range (back fixes Y, left fixes X, right
// sees nothing).
//
// we seed odom 4" off in X on purpose. watch the screen: pending X walks
// toward -4 as the cloud converges, then the flush snaps odom to the truth.
// after that it drives out and back, flushing only between motions.
void mcl_test() {
  constexpr double START_X = -48.0, START_Y = -48.0;  // true placement
  constexpr double ODOM_ERR_X = 4.0;                  // the planted error

  // lie to odom by 4". MCL gets seeded at the same wrong spot (that's all it
  // would know in a real match) and has to find the truth from the walls
  chassis.odom_xyt_set(START_X + ODOM_ERR_X, START_Y, 0.0);
  mcl::start(START_X + ODOM_ERR_X, START_Y, 3.0);
  mcl::auto_flush_set(false);  // manual flushes so each phase is visible

  // sit still, let the cloud converge
  pros::delay(4000);

  // flush -- odom X should jump ~-4"
  bool applied = mcl::flush_if_safe();
  printf("[mcl_test] flush #1 %s | odom (%.1f, %.1f) vs true (%.1f, %.1f)\n",
         applied ? "APPLIED" : "skipped",
         chassis.odom_x_get(), chassis.odom_y_get(), START_X, START_Y);
  pros::delay(1000);

  // drive out and back, flushing only between motions
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mcl::flush_if_safe();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mcl::flush_if_safe();

  // park with live numbers -- stop the program after reading
  while (true) {
    ez::screen_print("MCL  odom(" + util::to_string_with_precision(chassis.odom_x_get()) +
                         ", " + util::to_string_with_precision(chassis.odom_y_get()) + ")" +
                         "\nest (" + util::to_string_with_precision(mcl::x()) +
                         ", " + util::to_string_with_precision(mcl::y()) + ")" +
                         "\npend(" + util::to_string_with_precision(mcl::pending_x()) +
                         ", " + util::to_string_with_precision(mcl::pending_y()) + ")" +
                         "\nsprd(" + util::to_string_with_precision(mcl::spread_x()) +
                         ", " + util::to_string_with_precision(mcl::spread_y()) + ")" +
                         " conf " + (mcl::confident_x() ? "X" : "-") + (mcl::confident_y() ? "Y" : "-") +
                         "\nbeams ok " + std::to_string(mcl::sensors_accepted()) +
                         " gated " + std::to_string(mcl::sensors_gated()) +
                         " flushes " + std::to_string(mcl::flush_count()),
                     1);
    pros::delay(ez::util::DELAY_TIME);
  }
}
