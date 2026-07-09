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

///
// ============================================================================
//  overrideTest  --  match drive routine (encoder + IMU, no odom).
// ----------------------------------------------------------------------------
//  Intake on, drive in / turn / reverse to score, drop the hood, then
//  reposition (turns + drives) and deploy the wing.  Uses pid_drive_set /
//  turn_right_90, so distances come from the drive encoders and turns from the
//  IMU -- place the robot, no seeding needed.  Tune by editing the per-move
//  inches and the pros::delay holds.
// ============================================================================
///
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

  // These runtime values are lost on restart, so print them and keep them on the
  // brain screen.  Hard-code the matching number into your tracking wheel
  // constructor in main.cpp (for us, vert_tracker is the LEFT offset).
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

///
// Precise 90-degree IMU turns.
// pid_turn_relative_set turns relative to the robot's CURRENT heading using the
// inertial sensor, and pid_wait holds until the turn PID settles inside the exit
// conditions -- so these land on a tight 90 every time.
///
void turn_right_90(int speed) {
  chassis.pid_turn_relative_set(90_deg, speed);  // +90 = clockwise
  chassis.pid_wait();
}

void turn_left_90(int speed) {
  chassis.pid_turn_relative_set(-90_deg, speed);  // -90 = counter-clockwise
  chassis.pid_wait();
}

///
// Turn accuracy test: turn right 90 from a zeroed IMU, then display the actual
// heading and the error from a perfect 90.  If the PID is good, "Actual" reads
// ~90.0 and "Error" reads ~0.0.  Read it off the brain screen / PROS terminal.
///
void turn_test() {
  chassis.drive_imu_reset();              // zero the IMU so we measure from 0
  chassis.pid_targets_reset();

  turn_right_90();                        // the precise IMU turn we're testing

  double actual = chassis.drive_imu_get();  // true heading from the inertial sensor
  double error = actual - 90.0;             // how far off a perfect 90 we landed
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

// The tracking wheels live in main.cpp; pull them in for the direction test.
extern ez::tracking_wheel vert_tracker;
extern ez::tracking_wheel horiz_tracker;

///
// Tracking-wheel DIRECTION check (do this BEFORE measuring offsets).
// Push the robot by hand and watch the numbers:
//   push FORWARD -> VERT should count UP
//   push RIGHT   -> HORIZ should count UP
// If either counts DOWN, reverse that tracker's port (make it negative) in
// main.cpp, re-upload, and check again.  Only once BOTH count up should you
// run Measure Offsets.
///
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

///
// Clean offset check: the robot PIVOTS IN PLACE under motor power (stays
// centered -- no hand-sliding to contaminate it), then reports how far odom
// thinks x/y moved.  A perfect in-place spin should leave x/y ~0.
//   small x/y drift  -> horizontal offset is close, just nudge it
//   big x/y runaway  -> offset magnitude/sign is wrong
///
void odom_spin_test() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  chassis.drive_set(45, -45);   // pivot in place (left fwd, right rev)
  pros::delay(4000);            // ~several rotations
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

///
// Simplest possible odom move: from origin facing +Y, drive straight to (0,24).
// The robot should roll FORWARD 24" and end at ~(0, 24, 0).  If it drives
// BACKWARD, odom thinks "forward" is the wrong way -- a heading/vertical-tracker
// sign problem, not the path.
///
void odom_drive_test() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // origin, facing +Y (forward)
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

///
// Drift demo -- real sliding motion.
// Drift = one side of the drive at high power, the other at minimal power, so
// the bot carves a wide arc and the wheels slide.  Brakes are set to COAST so
// nothing fights the slide.  The intake runs both directions along the way.
//   high    = power on the fast side (out of 127)
//   low     = minimal power on the slow side (this is what makes it "slide")
//   sweepMs = how long to hold each drift -- bigger = wider, less compact
///
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

///
// BC2145AUTO -- controlled drift.
//   drift_demo(high side, low side, how long to hold each drift in ms)
//   <-- raise the 1100 (or lower the 30) for bigger, looser slides
///
void BC2145AUTO() {
  drift_demo(110, 30, 1100);
}

///
// BC2145AUTO_fullsend -- full send: max power, near-zero on the slow side, longer holds.
///
void BC2145AUTO_fullsend() {
  drift_demo(127, 15, 1400);
}

///
// JERRYIO PATH EXAMPLE
// Runs a path designed on https://path.jerryio.com using EZ-Template's
// pure-pursuit follower.  The path file lives in static/path.jerryio.txt and is
// compiled into the program (no SD card).
//
// To use your own path: export it, drop static/<name>.txt, add `PATH_ASSET
// <name>` to src/path_assets.S, declare it with ASSET(<name>_txt), then follow
// it.  Run `pros make clean` once after adding/changing a .txt file.
///
// The file name has dots, so it's embedded under the symbol-safe name
// `path_jerryio` (see PATH_ASSET path_jerryio, "path.jerryio.txt" in the .S).
ASSET(pushbackv2_txt);  // <-> static/pushbackv2.txt (via src/path_assets.S)

void jerryio_path_example() {
  // Intake ON for the whole path (both motors, stays running).
  bottom_intake_left.move(127);
  top_intake_right.move(127);

  // Start at the origin FACING FORWARD.  (The path is rotated to start forward,
  // so just place the robot facing forward -- no measuring the start angle.)
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // Drive FORWARD out to the turnaround, then REVERSE the output into the goal.
  // No 180 turn -- backing in along the return leg puts the output at the goal.
  jerryio::follow_path_reverse_tail(pushbackv2_txt, DRIVE_SPEED);
  chassis.pid_wait();

  // Backed into the goal -- drop the hood down.  (extend() = up, retract() = down.)
  // hood.retract();
}



// . . .
// Make your own autonomous functions here!
// . . .

void pistons() {
  descore.extend(); pros::delay(500);


}

///
// ODOM SANDBOX  (rewritten to use the proven "Odom Drive" convention)
//
// Start at the ORIGIN facing FORWARD (+Y) -- same setup as the working Odom
// Drive test, so the FIRST leg is a plain straight drive (no 90-degree seed).
//
// PLACEMENT: put the robot at the start, pointed the way it will first drive.
//   heading: 0 = forward (+Y), 90 = right (+X), 180 = back, 270 = left.
//
// It drives a 48" square CLOCKWISE and returns home to (0, 0):
//
//        (0,48) ---------> (48,48)
//          ^                  |
//   leg1 (fwd)                | leg2 (right)
//          |                  v
//   START (0,0) <-------- (48,0)
//             leg4 (right)   leg3 (right)
//
// Perfect tracking ends back at ~(0, 0).  The gap from (0,0) is your drift.
///
void odom_sandbox() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);  // origin, facing +Y (forward)

  // ===================== play area =====================
  chassis.pid_odom_set({{0_in, 48_in}, fwd, DRIVE_SPEED}, true);    // leg 1: forward to (0,48)
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 48_in}, fwd, DRIVE_SPEED}, true);   // leg 2: turn right -> (48,48)
  chassis.pid_wait();

  chassis.pid_odom_set({{48_in, 0_in}, fwd, DRIVE_SPEED}, true);    // leg 3: turn right -> (48,0)
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in}, fwd, DRIVE_SPEED}, true);     // leg 4: turn right -> home (0,0)
  chassis.pid_wait();
  // =====================================================

  // Report the final pose.  Home is (0, 0), so perfect tracking reads ~0, 0.
  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();
  double t = chassis.odom_theta_get();
  printf("Odom sandbox end -> x: %f  y: %f  theta: %f  (home is 0,0)\n", x, y, t);
}

///
// MOTION PROFILE TEST
// Drives straight using the trapezoidal motion profile (src/motion_profile.cpp)
// instead of EZ-Template's PID, then prints how far it actually traveled vs the
// 48" target so you can see how well it tracks.  Tune kV/kA/kP in that file.
///
void motion_profile_test() {
  // profiled_drive(distance_in, max_vel_in_per_s, max_accel_in_per_s2)
  profiled_drive(48, 30, 60);

  double traveled = (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;
  printf("Motion profile -> target: 48.0 in   traveled: %f in\n", traveled);
}

///
// WALL RESET TEST
// Square the robot against a known wall, run this, and watch the printed pose
// jump to the true value.  It deliberately mis-seeds odom first (so there's
// "drift" to correct), prints, runs wall_reset_all(), then prints the result.
///
void wall_reset_test() {
  // Pretend odom has drifted: seed a deliberately-wrong pose.
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  printf("Wall reset -- BEFORE: x:%.2f y:%.2f t:%.2f\n",
         chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());

  wall_reset_all();  // each sensor squared to a wall corrects its axis

  printf("Wall reset -- AFTER:  x:%.2f y:%.2f t:%.2f\n",
         chassis.odom_x_get(), chassis.odom_y_get(), chassis.odom_theta_get());
}

///
// PROFILING SHOWCASE
// Shows where motion profiling shines: smooth, planned straight-line moves.
// It drives a square (~30" sides, stays inside 50x50") alternating FAST sides
// and GENTLE sides so you can SEE the difference:
//   - FAST sides ramp up hard, cruise, then brake smoothly to a clean stop --
//     no PID overshoot, even at speed.
//   - GENTLE sides use low acceleration (jerk-limited) -- they ease in and out
//     with no lurch.  This is the move you'd want on a tall, tippy robot.
// The STRAIGHTS are profiled (src/motion_profile.cpp).  The 90-deg corners use
// EZ-Template's turn (turning isn't profiled here).  Pauses between moves let
// you watch each one settle.
///
void profiling_showcase() {
  const int LEG = 30;  // inches per side -- keeps the path well inside 50x50

  // Side 1 -- FAST: high speed, planned decel = crisp stop
  profiled_drive(LEG, 40, 80);
  pros::delay(700);
  turn_right_90();

  // Side 2 -- GENTLE: low accel, jerk-limited (anti-tip / careful with a load)
  profiled_drive(LEG, 16, 16);
  pros::delay(700);
  turn_right_90();

  // Side 3 -- FAST again
  profiled_drive(LEG, 40, 80);
  pros::delay(700);
  turn_right_90();

  // Side 4 -- GENTLE back to the start corner
  profiled_drive(LEG, 16, 16);
  pros::delay(700);
  turn_right_90();

  // Bonus -- a SHORT move shows the profile adapt: too short to reach top speed,
  // so it becomes a triangle (ramp up -> ramp straight down), still smooth.
  // Out and back so we stay put.
  profiled_drive(12, 40, 80);
  pros::delay(400);
  profiled_drive(-12, 40, 80);
}

///
// ============================================================================
//  MCL Test  --  prove the particle filter end-to-end.
// ----------------------------------------------------------------------------
//  SETUP (tape measure, one time): place the robot FACING FORWARD (theta 0)
//  at field (-48, -48), bottom-left, so that:
//    - the BACK wall is ~22" behind robot center  (back sensor reads ~16")
//    - the LEFT wall is ~22" left of robot center (left sensor reads ~16")
//  Both sensors in range (back fixes Y, left fixes X); the right sensor sees
//  nothing (>36" away).  Two walls -> both axes correct.
//
//  The test seeds odom 4" OFF on purpose (X only).  Watch the screen:
//    1. pending X correction walks from 0 toward -4.0 as the cloud converges
//       (Y pending stays ~0 -- the back wall says Y is fine).
//    2. After the flush, odom X snaps to the truth; pending returns to ~0.
//    3. Then it drives forward 24" and back, flushing at the pid_wait()
//       boundary -- corrections between motions, never mid-servo.
// ============================================================================
///
void mcl_test() {
  constexpr double START_X = -48.0, START_Y = -48.0;  // true placement (field in)
  constexpr double ODOM_ERR_X = 4.0;                  // deliberate seeded error

  // Lie to odom by 4" in X.  MCL is seeded at the same wrong pose (that's all
  // it would know in a real match) and has to FIND the truth from the walls.
  chassis.odom_xyt_set(START_X + ODOM_ERR_X, START_Y, 0.0);
  mcl::start(START_X + ODOM_ERR_X, START_Y, 3.0);
  mcl::auto_flush_set(false);  // manual flushes here so each phase is visible

  // Phase 1: sit still and let the cloud converge on the walls.
  pros::delay(4000);

  // Phase 2: flush at a safe point -- odom X should jump ~-4".
  bool applied = mcl::flush_if_safe();
  printf("[mcl_test] flush #1 %s | odom (%.1f, %.1f) vs true (%.1f, %.1f)\n",
         applied ? "APPLIED" : "skipped",
         chassis.odom_x_get(), chassis.odom_y_get(), START_X, START_Y);
  pros::delay(1000);

  // Phase 3: drive fwd 24" and back; flush only BETWEEN motions.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mcl::flush_if_safe();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  mcl::flush_if_safe();

  // Park with live telemetry -- stop the program after reading.
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
