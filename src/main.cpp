#include "main.h"

// built on EZ-Template, docs at https://ez-robotics.github.io/EZ-Template/

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

  // ODOM DISABLED: no tracking wheels on this bot (they'd go on ports 6/15). The
  // IMU (port 9) still gives pid_drive_set / pid_turn_set their heading, so those
  // work off the IMU + motor encoders. Full 2D odom (pid_odom_set) needs trackers,
  // so mount them, then re-enable these two lines.
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // chassis.odom_tracker_left_set(&vert_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);   // adjust curve with the joystick buttons
  chassis.opcontrol_drive_activebrake_set(0.0);   // 0 = off, EZ recommends ~2
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  // arm holds its position when no button is pressed instead of coasting down
  arm.set_brake_mode_all(pros::MotorBrake::hold);
  arm.tare_position_all();  // zero the arm encoders

  // auton selector (brain screen). first entry = default
  ez::as::auton_selector.autons_add({
      {"Override Test\n\nMatch auton: drive + turns + arm (EZ IMU PID)", overrideTest},
      {"Skills\n\nProgramming skills run (60s solo)", skills},
      {"Arm Height Test\n\nCycles the arm through low/mid/high presets (move_absolute), prints encoder pos", arm_height_test},
      {"PID Square\n\n48in square (straights + 90 turns), tune drive/turn PID by how well it closes", pid_square},
      {"Motion Profile\n\nTrapezoidal profiled drive 48in, print traveled distance", motion_profile_test},
      {"Odom Spin\n\nMotor pivots in place; reports x/y drift (should be ~0)", odom_spin_test},
      {"MCL Test\n\nParticle filter: converge near corner, catch a planted 4in odom error, flush between motions", mcl_test},
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

    arm_control();     // L1 = arm up / L2 = arm down
    intake_control();  // R1 = intake in / R2 = intake out

    pros::delay(ez::util::DELAY_TIME);  // keep this, EZ uses it for timing
  }
}
