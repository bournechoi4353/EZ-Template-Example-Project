#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// ports + measurements are all in config.hpp. first motor per side does the
// sensing
ez::Drive chassis(
    LEFT_DRIVE_PORTS,
    RIGHT_DRIVE_PORTS,
    IMU_PORT,
    WHEEL_DIAMETER,
    DRIVE_RPM);

ez::tracking_wheel horiz_tracker(HORIZ_TRACKER_PORT, HORIZ_TRACKER_DIAMETER, HORIZ_TRACKER_OFFSET);
ez::tracking_wheel vert_tracker(VERT_TRACKER_PORT, VERT_TRACKER_DIAMETER, VERT_TRACKER_OFFSET);

// runs as soon as the program starts
void initialize() {
  ez::ez_template_print();

  pros::delay(500);  // let the ports configure before anything runs

  // horiz is behind the midline -> back, vert is left of center -> left.
  // setting both enables full 2D odom
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);   // adjust curve with the joystick buttons
  chassis.opcontrol_drive_activebrake_set(0.0);   // 0 = off, EZ recommends ~2
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  // descore stays up the whole match -- deploy at startup, never lower it
  descore.extend();

  // auton selector (brain screen). first entry = default
  ez::as::auton_selector.autons_add({
      {"overrideTest\n\nMatch drive routine: intake, score, hood, reposition, wing (encoder + IMU).", overrideTest},
      {"JerryIO Path\n\nRun pushbackv2.txt via pure pursuit (intake on, hood drops at end)", jerryio_path_example},
      {"BC2145AUTO\n\nSmooth drift demo (contained)", BC2145AUTO},
      {"Full Send\n\nSame drift path, max slide", BC2145AUTO_fullsend},
      {"pistons\n\nDemo: pistons test", pistons},
      {"Odom Sandbox\n\nPlay with odometry -- drive to coordinates, print final pose", odom_sandbox},
      {"Motion Profile\n\nTrapezoidal profiled drive 48in, print traveled distance", motion_profile_test},
      {"Profiling Showcase\n\nFast vs gentle profiled straights in a 50x50 box", profiling_showcase},
      {"Wall Reset\n\nSquare to a wall; snap odom X/Y from the distance sensors", wall_reset_test},
      {"MCL Test\n\nParticle filter: converge near corner, catch a planted 4in odom error, flush between motions", mcl_test},
      {"Turn Test\n\nTurn right 90, show actual heading + error", turn_test},
      {"Tracker Dir\n\nPush robot: forward=vert up, right=horiz up (verify before Measure Offsets)", tracker_dir_test},
      {"Odom Spin\n\nMotor pivots in place; reports x/y drift (should be ~0)", odom_spin_test},
      {"Odom Drive\n\nDrive straight fwd 24in via odom; should end ~(0,24,0)", odom_drive_test},
      {"Measure Offsets\n\nTurns the robot a bunch of times and calculates your tracking-wheel offsets.", measure_offsets},
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

// runs while disabled at comp
void disabled() {
  // . . .
}

// runs before auton when connected to field control
void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // hold helps consistency

  ez::as::auton_selector.selected_auton_call();
}

// print a tracker's value + width on the brain screen
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

// extra debug pages on the brain screen (only off comp)
void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      // odom debug page: pose + live tracker values
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // no debug pages when on comp control
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

// EZ extras, only active off comp control: X toggles the PID tuner
// (A/Y change values, arrows navigate), DOWN+B runs the selected auton
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    chassis.pid_tuner_iterate();
  }

  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

// driver control
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();

    chassis.opcontrol_tank();
    // chassis.opcontrol_arcade_standard(ez::SPLIT);
    // chassis.opcontrol_arcade_standard(ez::SINGLE);
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);

    intake_control();  // R1 in / R2 out / L1 hood

    pros::delay(ez::util::DELAY_TIME);  // keep this, EZ uses it for timing
  }
}
