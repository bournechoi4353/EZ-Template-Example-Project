#include "main.h"
#include "brain_screen.hpp"

// built on EZ-Template, docs at https://ez-robotics.github.io/EZ-Template/

// ports + measurements are all in ports.hpp. first motor per side does the
// sensing
ez::Drive chassis(
    LEFT_DRIVE_PORTS,
    RIGHT_DRIVE_PORTS,
    IMU_PORT,
    WHEEL_DIAMETER,
    DRIVE_RPM);

// LEFT/RIGHT on the controller cycle ONLY the real match autons (the other
// entries in autons_add() are test/debug tools, not used in competition).
// must be the first N entries of autons_add(), in the same order.
std::vector<std::string> auton_names = {
    "redLeft",
    "redRight",
    "blueLeft",
    "blueRight",
};

// runs as soon as the program starts
void initialize() {
  ez::ez_template_print();

  pros::delay(500);  // let the ports configure before anything runs

  // no tracking wheels on this bot -- forward distance comes from the drive
  // motor encoders, heading from the IMU. pid_drive_set/pid_turn_set work off
  // those either way.

  chassis.opcontrol_curve_buttons_toggle(true);   // adjust curve with the joystick buttons
  chassis.opcontrol_drive_activebrake_set(0.0);   // 0 = off, EZ recommends ~2
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  // arm holds its position when no button is pressed instead of coasting down
  arm.set_brake_mode(pros::MotorBrake::hold);
  arm.tare_position();  // zero the arm encoder
  arm_sensor.reset_position();  // zero the arm hard-stop reference -- wherever the arm rests at boot = 0

  // auton selector (brain screen). first entry = default
  ez::as::auton_selector.autons_add({
      {"redLeft", redLeft},
      {"redRight", redRight},
      {"blueLeft", blueLeft},
      {"blueRight", blueRight},
      {"Skills\n\nProgramming skills run", skills},
      {"Arm Height Capture\n\nJog the arm by hand (R2/L2), screen shows live position -- write down the number at each pin height", arm_height_capture},
  });

  chassis.initialize();

  // custom LVGL auton selector -- replaces ez::as::initialize()'s screen.
  // (EZ's autons_add above is left in place but its screen is no longer shown.)
  brain_screen_init();

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

// runs while disabled at comp
void disabled() {
  // . . .
}

// runs before auton when connected to field control. LEFT/RIGHT on the
// controller cycle ONLY the real match autons (the first auton_names.size()
// entries of the brain selector, in the same order) so drivers can't land on a
// test routine at comp. The brain touchscreen can still reach everything.
// Reads/writes brain_screen.selected_index() directly -- no separate index to
// go stale, so whatever the brain is showing is exactly what autonomous()
// will run.
void competition_initialize() {
  while (true) {
    bool left = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool right = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

    if (left || right) {
      int idx = brain_screen.selected_index();
      // if the brain is currently on a test/debug auton (off comp control
      // testing left it there), snap into match range first
      if (idx >= (int)auton_names.size()) idx = right ? -1 : 0;

      if (left) idx = (idx - 1 + auton_names.size()) % auton_names.size();
      else idx = (idx + 1) % auton_names.size();

      brain_screen.select(idx);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // hold helps consistency

  brain_screen.run_selected();
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
// NOTE: disabled in favor of custom brain_screen above
// Uncomment below to re-enable EZ template debug pages
/*
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
*/

// EZ extras, only active off comp control: DOWN+B runs the selected auton
// (note: controls() also uses B for roller-out, so holding this combo spins
// the roller too -- harmless off comp control, but worth knowing).
// PID tuner is disabled -- X is free.
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      master.rumble(".");  // TEMP: confirms the combo was actually detected
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }
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

    controls();  // every driver-control button binding -- see controls.cpp

    pros::delay(ez::util::DELAY_TIME);  //EZ uses this for timing
  }
}


