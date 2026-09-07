#include "main.h"

#include <cmath>

// trapezoidal profile for straight drives. standalone, doesn't touch
// EZ-Template, just commands the motors with drive_set().
//
// PID only reacts to error. this plans the whole move up front (ramp up,
// cruise, ramp down) and every 10ms commands
//   power = kV*planned_vel + kA*planned_accel + kP*(planned_pos - actual)
// plus a heading hold so it goes straight.
//
// tuning: set kV/kA to 0 first and raise kP until it follows the trapezoid
// without lagging (lower if it oscillates). then raise kV until kP barely
// has to correct, roughly cruise power / max vel. add a little kA only if
// it lags on the ramp.

static int clamp127(double v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int)v;
}

void profiled_drive(double target_in, double max_vel, double max_accel) {
  const double kV = 2.0;    // power per in/s of planned velocity
  const double kA = 0.2;    // power per in/s^2 of planned accel
  const double kP = 6.0;    // power per inch of position error
  const double kP_h = 3.0;  // heading hold, power per degree off

  const double dt = ez::util::DELAY_TIME / 1000.0;  // 0.01s per loop

  chassis.drive_sensor_reset();
  chassis.drive_imu_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  double dir = (target_in >= 0) ? 1.0 : -1.0;
  double dist = std::fabs(target_in);

  // build the trapezoid (or a triangle if the move is too short to hit max_vel)
  double t_accel = max_vel / max_accel;
  double d_accel = 0.5 * max_accel * t_accel * t_accel;
  double v_peak = max_vel;
  double t_cruise;
  if (2.0 * d_accel >= dist) {
    t_accel = std::sqrt(dist / max_accel);
    d_accel = 0.5 * max_accel * t_accel * t_accel;
    v_peak = max_accel * t_accel;
    t_cruise = 0.0;
  } else {
    t_cruise = (dist - 2.0 * d_accel) / max_vel;
  }
  double t_total = 2.0 * t_accel + t_cruise;

  double t = 0.0;
  while (t <= t_total + 0.25) {  // extra 0.25s to settle after the plan ends
    double des_pos, des_vel, des_acc;
    if (t < t_accel) {                      // ramp up
      des_acc = max_accel;
      des_vel = max_accel * t;
      des_pos = 0.5 * max_accel * t * t;
    } else if (t < t_accel + t_cruise) {    // cruise
      des_acc = 0.0;
      des_vel = v_peak;
      des_pos = d_accel + v_peak * (t - t_accel);
    } else if (t < t_total) {               // ramp down
      double td = t - t_accel - t_cruise;
      des_acc = -max_accel;
      des_vel = v_peak - max_accel * td;
      des_pos = d_accel + v_peak * t_cruise + (v_peak * td - 0.5 * max_accel * td * td);
    } else {                                // plan done, hold the end
      des_acc = 0.0;
      des_vel = 0.0;
      des_pos = dist;
    }

    double actual = dir * (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;

    double power = kV * des_vel + kA * des_acc + kP * (des_pos - actual);
    power *= dir;

    // heading hold at 0. if it curves instead of going straight, flip kP_h's sign
    double turn = kP_h * (0.0 - chassis.drive_imu_get());

    chassis.drive_set(clamp127(power + turn), clamp127(power - turn));
    pros::delay(ez::util::DELAY_TIME);
    t += dt;
  }

  chassis.drive_set(0, 0);
  chassis.pid_targets_reset();  // hand a clean slate back to EZ's PID
}
