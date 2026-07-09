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

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 120.0);         // Fwd/rev (odom + non-odom); kD raised to brake the overshoot
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

// main match auton. everything here is drive encoders + IMU (no odom), so
// just place the robot and go. tune the inches and delays below.
void overrideTest() {

  //toggle
  chassis.pid_drive_set(7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);

  // Turn right 45 degrees
  turn_right_45();

  // Forward 34"
  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //checkpoint 1
  mcl_checkpoint(-24, -48);

  // place pin
  top_intake_right.move(127);
  

}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);

  // these are lost on restart -- write them down and hard-code them into the
  // tracker constructors in main.cpp (vert_tracker uses the Left number)
  printf("Offsets -> Left: %f  Right: %f  Back: %f  Front: %f\n", l_offset, r_offset, b_offset, f_offset);
  while (true) {
    ez::screen_print("Tracker offsets (put in main.cpp):"
                         "\nLeft (vert):  " + util::to_string_with_precision(l_offset) +
                         "\nRight: " + util::to_string_with_precision(r_offset) +
                         "\nBack:  " + util::to_string_with_precision(b_offset) +
                         "\nFront: " + util::to_string_with_precision(f_offset),
                     1);
    pros::delay(ez::util::DELAY_TIME);
  }
}

// tight IMU 90s, relative to wherever we're currently facing
void turn_right_90(int speed) {
  chassis.pid_turn_relative_set(90_deg, speed);  // +90 = clockwise
  chassis.pid_wait();
}

void turn_left_90(int speed) {
  chassis.pid_turn_relative_set(-90_deg, speed);  // -90 = counter-clockwise
  chassis.pid_wait();
}

// turn right 90 from a zeroed IMU and show how far off we landed.
// if the turn PID is good, Error reads ~0
void turn_test() {
  chassis.drive_imu_reset();
  chassis.pid_targets_reset();

  turn_right_90();

  double actual = chassis.drive_imu_get();
  double error = actual - 90.0;
  printf("Turn test -> Target: 90.0  Actual: %f  Error: %f\n", actual, error);
  while (true) {
    ez::screen_print("Turn accuracy test:"
                         "\nTarget: 90.0"
                         "\nActual: " + util::to_string_with_precision(actual) +
                         "\nError:  " + util::to_string_with_precision(error),
                     1);
    pros::delay(ez::util::DELAY_TIME);
  }
}

// trackers live in main.cpp, pull them in for the direction test
extern ez::tracking_wheel vert_tracker;
extern ez::tracking_wheel horiz_tracker;

// tracker direction check -- run this BEFORE measure offsets. push forward:
// vert counts up. push right: horiz counts up. if one counts down, flip its
// port sign in main.cpp and check again. a backwards tracker poisons the
// measured offsets.
void tracker_dir_test() {
  vert_tracker.reset();
  horiz_tracker.reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  while (true) {
    ez::screen_print("fwd: VERT up, horiz ~0 | right: HORIZ up | SPIN: x/y stay"
                         "\nvert:  " + util::to_string_with_precision(vert_tracker.get()) +
                         "   horiz: " + util::to_string_with_precision(horiz_tracker.get()) +
                         "\nx: " + util::to_string_with_precision(chassis.odom_x_get()) +
                         "   y: " + util::to_string_with_precision(chassis.odom_y_get()) +
                         "   t: " + util::to_string_with_precision(chassis.odom_theta_get()),
                     1);
    pros::delay(ez::util::DELAY_TIME);
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

// simplest odom move there is: drive straight to (0, 24). if the robot goes
// BACKWARD instead, odom thinks forward is the wrong way -- that's a
// heading/tracker sign problem, not the path.
void odom_drive_test() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // origin, facing forward
  chassis.pid_odom_set({{0_in, 24_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();

  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();
  double t = chassis.odom_theta_get();
  printf("Odom drive -> x: %.2f  y: %.2f  t: %.2f  (robot should go FWD, end ~0,24,0)\n", x, y, t);
  while (true) {
    ez::screen_print("Simple odom drive (fwd 24in):"
                         "\n x: " + util::to_string_with_precision(x) +
                         "\n y: " + util::to_string_with_precision(y) +
                         "\n t: " + util::to_string_with_precision(t),
                     1);
    pros::delay(ez::util::DELAY_TIME);
  }
}

// drift demo. one side at high power, the other barely moving, coast brakes
// so nothing fights the slide. high/low = power per side, sweepMs = how long
// each arc is held (bigger = wider).
void drift_demo(int high, int low, int sweepMs) {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);  

  set_bottom_intake(100);  // intake running

  chassis.drive_set(high, high);
  pros::delay(600);

  chassis.drive_set(high, low);
  pros::delay(sweepMs);
  wing.extend();  

  chassis.drive_set(high, -high);
  pros::delay(350);

  set_bottom_intake(-100);  
  chassis.drive_set(low, high);
  pros::delay(sweepMs);
  loader.extend();   

  descore.extend(); 
  chassis.drive_set(-high, high);
  pros::delay(350);


  chassis.drive_set(high, low);
  pros::delay(sweepMs + 300);
  lift.extend();  
  chassis.drive_set(high, -high);
  pros::delay(350);

 
  set_bottom_intake(100);
  chassis.drive_set(low, high);
  pros::delay(sweepMs);
  hood.extend();  // pop the hood

 
  chassis.drive_set(0, 0);
  pros::delay(400);
  set_bottom_intake(0);

  wing.retract();
  loader.retract();  lift.retract();
  hood.retract();

 
  chassis.drive_sensor_reset();
  chassis.pid_targets_reset();
}

// contained drift. raise the 1100 (or lower the 30) for bigger looser slides
void BC2145AUTO() {
  drift_demo(110, 30, 1100);
}

// same thing, max power
void BC2145AUTO_fullsend() {
  drift_demo(127, 15, 1400);
}

// runs a path.jerryio path with EZ's pure pursuit. the file is compiled in
// from static/ -- remember `pros make clean` after touching a .txt
ASSET(pushbackv2_txt);  // static/pushbackv2.txt

void jerryio_path_example() {
  // intake on for the whole path
  bottom_intake_left.move(127);
  top_intake_right.move(127);

  // path is rotated to start forward, so just place the robot facing forward
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // forward to the turnaround, then back the output into the goal (no 180)
  jerryio::follow_path_reverse_tail(pushbackv2_txt, DRIVE_SPEED);
  chassis.pid_wait();

  // hood.retract();  // drop the hood once we're backed in
}



// . . .
// Make your own autonomous functions here!
// . . .

void pistons() {
  descore.extend(); pros::delay(500);


}

// odom playground. start at the origin facing forward (same setup as the
// odom drive test), drives a 48" square clockwise and comes home to (0,0).
// whatever gap is left at the end is your drift.
void odom_sandbox() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_odom_set({{0_in, 48_in}, fwd, DRIVE_SPEED}, true);    // up
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 48_in}, fwd, DRIVE_SPEED}, true);   // right
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 0_in}, fwd, DRIVE_SPEED}, true);    // down
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in}, fwd, DRIVE_SPEED}, true);     // home
  chassis.pid_wait();

  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();
  double t = chassis.odom_theta_get();
  printf("Odom sandbox end -> x: %f  y: %f  theta: %f  (home is 0,0)\n", x, y, t);
}

// straight 48" with the trapezoid profile instead of PID, then prints how
// far it actually went. tune kV/kA/kP in motion_profile.cpp
void motion_profile_test() {
  // profiled_drive(inches, max vel in/s, max accel in/s^2)
  profiled_drive(48, 30, 60);

  double traveled = (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;
  printf("Motion profile -> target: 48.0 in   traveled: %f in\n", traveled);
}

// square the robot up against a known wall and run this -- the pose should
// jump to the true value. we seed a wrong pose on purpose so there's
// something to correct.
void wall_reset_test() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // deliberately wrong
  printf("Wall reset -- BEFORE: x:%.2f y:%.2f t:%.2f\n",
         chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  wall_reset_all();

  printf("Wall reset -- AFTER:  x:%.2f y:%.2f t:%.2f\n",
         chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
}

// drives a square alternating fast and gentle profiled sides so you can see
// the difference: fast = hard ramp + clean stop with no PID overshoot,
// gentle = low accel, no lurch (what you'd want on a tall tippy robot).
// straights are profiled, corners are normal EZ turns.
void profiling_showcase() {
  const int LEG = 30;  // stays inside 50x50

  // fast
  profiled_drive(LEG, 40, 80);
  pros::delay(700);
  turn_right_90();

  // gentle
  profiled_drive(LEG, 16, 16);
  pros::delay(700);
  turn_right_90();

  // fast
  profiled_drive(LEG, 40, 80);
  pros::delay(700);
  turn_right_90();

  // gentle, back to the start corner
  profiled_drive(LEG, 16, 16);
  pros::delay(700);
  turn_right_90();

  // bonus: too short to hit top speed, so the profile turns into a triangle.
  // out and back so we stay put
  profiled_drive(12, 40, 80);
  pros::delay(400);
  profiled_drive(-12, 40, 80);
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
