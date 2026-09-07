#include "main.h"
#include "brain_screen.hpp"

// built on EZ-Template, docs at https://ez-robotics.github.io/EZ-Template/

// all out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 127;

// milliseconds to wait between auton movements. change this one number; 0 = none
const int PAUSE = 0;

// drive / turn / odom PID tuning, everything EZ needs, set once at startup
void default_constants() {
  // kP, kI, kD (turn also takes a start-I)
  chassis.pid_drive_constants_set(12.0, 0.0, 80.0);          // fwd/rev, lower kP eases into the target instead of braking hard
  chassis.pid_heading_constants_set(6.0, 0.0, 20.0);
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // swings, one side pivots
  chassis.pid_odom_angular_constants_set(4.0, 0.0, 52.5);    // heading for odom moves, lowered kP to kill the side-to-side sway
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // heading for boomerang moves

  // when a move counts as "done"
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // slew, ramp up so we don't jerk off the line
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(70_deg, 40);  // start at 40, ramp toward whatever max_speed you pass per-call
  // how much turning wins over driving in odom moves (1.0 = max).
  // bump it up if you add tracking wheels
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(8_in);           // how far ahead it looks, smaller = less overshoot at the end, bigger = smoother
  chassis.odom_boomerang_distance_set(16_in);  // furthest the carrot point can sit from the target
  chassis.odom_boomerang_dlead_set(0.625);     // how aggressive the tail of a boomerang move is

  chassis.pid_angle_behavior_set(ez::shortest);  // default turns take the shortest way around
}

void redLeft() {
  chassis.pid_drive_set(-5_in, 127, true);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 127, true);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, true);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 60_deg, 90);
  chassis.pid_wait();

  arm.move_absolute(350, 100);

  chassis.pid_drive_set(-14_in, 127, true);
  chassis.pid_wait();
  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  pros::delay(700);
  set_arm(0);
  chassis.pid_swing_relative_set(ez::LEFT_SWING, 67_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-13_in, 30, false);
  chassis.pid_wait();
  chassis.pid_swing_relative_set(ez::RIGHT_SWING, 42_deg, 70);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 30, false);
  chassis.pid_wait();
  set_arm(-40);
  pros::delay(800);
  clawPiston.retract();
  arm.move_absolute(900, 100);

  chassis.pid_swing_relative_set(ez::LEFT_SWING, -90_deg, 40);
  chassis.pid_wait();
  chassis.pid_drive_set(15_in, 70, false);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-40_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-10_in, 70, false);
  chassis.pid_wait();
  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  chassis.pid_drive_set(5_in, 70, false);
  chassis.pid_wait();

}

void redRight() {
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();

  arm.move_absolute(350, 100);
  chassis.pid_drive_set(-18_in, 127, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in, 60, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, 60, true);
  chassis.pid_wait();

  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  pros::delay(700);
  set_arm(0);


  chassis.pid_drive_set(20_in, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-45_deg, 120);
  chassis.pid_wait();
  arm.move_absolute(-20, 100);
  chassis.pid_drive_set(-14_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-30.5_in, 50, true);
  chassis.pid_wait();


}

void blueLeft() {
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();

  arm.move_absolute(350, 100);
  chassis.pid_drive_set(-18_in, 127, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in, 60, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, 60, true);
  chassis.pid_wait();

  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  pros::delay(700);
  set_arm(0);


  chassis.pid_drive_set(20_in, 90, true);
  chassis.pid_wait();
  
  chassis.pid_turn_relative_set(-44.5_deg, 120);
  chassis.pid_wait();
  arm.move_absolute(-20, 100);
  chassis.pid_drive_set(-29_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, 20, true);
  chassis.pid_wait();

  clawPiston.retract();
  arm.move_absolute(700, 100);
  pros::delay(700);

  chassis.pid_turn_relative_set(126_deg, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-14.5_in, 50, true);
  chassis.pid_wait();
  arm.move_absolute(305, 100);
  pros::delay(800);
  clawPiston.extend();
  chassis.pid_drive_set(5_in, 127, true);
  chassis.pid_wait();




}

void blueRight() {
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();

  arm.move_absolute(350, 100);
  chassis.pid_drive_set(-18_in, 127, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(-90_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in, 60, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, 60, true);
  chassis.pid_wait();

  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  pros::delay(700);
  set_arm(0);


  chassis.pid_drive_set(20_in, 90, true);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(-45_deg, 120);
  chassis.pid_wait();
  arm.move_absolute(-20, 100);
  chassis.pid_drive_set(-15_in, 50, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(45_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-30.5_in, 50, true);
  chassis.pid_wait();


}

// programming skills run
void skills() {
  chassis.pid_drive_set(-4_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(6_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, 127, false);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, 127, false);
  chassis.pid_wait();

  arm.move_absolute(350, 100);
  chassis.pid_drive_set(-23_in, 127, true);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(90_deg, 120);
  chassis.pid_wait();
  chassis.pid_drive_set(-19_in, 60, true);
  chassis.pid_wait();
  chassis.pid_drive_set(1.5_in, 60, true);
  chassis.pid_wait();

  set_arm(-40);
  pros::delay(800);
  clawPiston.extend();
  pros::delay(700);
  set_arm(0);


  chassis.pid_drive_set(20_in, 90, true);
  chassis.pid_wait();
  
  chassis.pid_turn_relative_set(-43_deg, 120);
  chassis.pid_wait();
  arm.move_absolute(-20, 100);
  chassis.pid_drive_set(-29.5_in, 65, true);
  chassis.pid_wait();
  chassis.pid_drive_set(-3.5_in, 20, true);
  chassis.pid_wait();

  clawPiston.retract();
  arm.move_absolute(700, 100);
  pros::delay(700);

  chassis.pid_turn_relative_set(123_deg, 90);
  chassis.pid_wait();

  chassis.pid_drive_set(-14.5_in, 50, true);
  chassis.pid_wait();
  arm.move_absolute(305, 100);
  pros::delay(800);
  clawPiston.extend();

  chassis.pid_drive_set(40_in, 127, true);
  chassis.pid_wait();
}

// live capture tool for pin heights. tares at 0 (resting position), sets the
// arm to COAST so it moves freely by hand (it's HOLD brake normally, which
// would fight you), then shows the live encoder position on screen -- push it
// to each pin count's real height and write down the number. R2/L2 still work
// too if you'd rather jog it than push it. no auto-stepping, runs forever;
// pick a different auton (or restart) to exit.
void arm_height_capture() {
  arm_sensor.reset_position();  // 0 = resting position (0 pins)
  arm.set_brake_mode(pros::MotorBrake::coast);  // free to push by hand

  while (true) {
    if (master.get_digital(DIGITAL_R2)) {
      set_arm(127);   // full up
    } else if (master.get_digital(DIGITAL_L2)) {
      set_arm(-127);  // full down
    } else {
      set_arm(0);
    }

    double position = arm_sensor.get_position() / 100.0;
    printf("Arm height capture -> position: %.1f\n", position);
    brain_screen.show_text("Arm height capture"
                                "\nposition: " + util::to_string_with_precision(position) +
                                "\n(push by hand, or R2 up / L2 down, 0 = rest)"
                                "\nmove to each pin height, write down the number");

    pros::delay(ez::util::DELAY_TIME);
  }
}
