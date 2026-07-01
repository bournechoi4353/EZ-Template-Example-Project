#include "main.h"

#include <cmath>

// ============================================================================
//  Trapezoidal motion profile -- straight-line drive
//  ------------------------------------------------------------------------
//  This is a self-contained add-on.  It does NOT change EZ-Template; it just
//  commands the drive motors directly with chassis.drive_set().
//
//  Unlike PID (which only reacts to error), a motion profile PLANS the whole
//  move ahead of time as a velocity curve:  ramp up -> cruise -> ramp down.
//  Every 10 ms we look up where we're SUPPOSED to be / how fast we're SUPPOSED
//  to be going, and command:
//
//     power = kV*desired_vel + kA*desired_accel   (feedforward: the plan)
//           + kP*(desired_pos - actual_pos)        (feedback: fix the error)
//
//  plus a heading-hold term so it tracks straight.
//
//  ---- TUNING (these are robot-specific guesses, dial them in) ----
//  Easiest path:
//    1. Set kV = kA = 0 first.  Now it's just a position controller chasing a
//       trapezoid -- raise kP until it follows the move without lag, lower it
//       if it oscillates.  This alone already demonstrates profiling.
//    2. Then add kV: raise it until the robot keeps up with the planned speed
//       on its own (kP barely has to correct).  kV ~= cruise_power / max_vel.
//    3. Add a little kA only if it lags during the ramp.
// ============================================================================

static int clamp127(double v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return (int)v;
}

void profiled_drive(double target_in, double max_vel, double max_accel) {
  // ---- Tunables (start here) ----
  const double kV = 2.0;    // power per (inch/sec) of planned velocity
  const double kA = 0.2;    // power per (inch/sec^2) of planned accel
  const double kP = 6.0;    // position-error correction (power per inch off)
  const double kP_h = 3.0;  // heading hold (power per degree off straight)

  const double dt = ez::util::DELAY_TIME / 1000.0;  // 0.01 s per loop

  chassis.drive_sensor_reset();
  chassis.drive_imu_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  double dir = (target_in >= 0) ? 1.0 : -1.0;
  double dist = std::fabs(target_in);

  // ---- Build the trapezoid (or triangle if the move is too short) ----
  double t_accel = max_vel / max_accel;
  double d_accel = 0.5 * max_accel * t_accel * t_accel;
  double v_peak = max_vel;
  double t_cruise;
  if (2.0 * d_accel >= dist) {
    // Never reaches max_vel -> triangular profile
    t_accel = std::sqrt(dist / max_accel);
    d_accel = 0.5 * max_accel * t_accel * t_accel;
    v_peak = max_accel * t_accel;
    t_cruise = 0.0;
  } else {
    t_cruise = (dist - 2.0 * d_accel) / max_vel;
  }
  double t_total = 2.0 * t_accel + t_cruise;

  // ---- Run the profile ----
  double t = 0.0;
  while (t <= t_total + 0.25) {  // +0.25 s to settle on the target after the plan ends
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
    } else {                                // plan finished -> hold the end
      des_acc = 0.0;
      des_vel = 0.0;
      des_pos = dist;
    }

    // Progress along the move (positive as it goes toward the target)
    double actual = dir * (chassis.drive_sensor_left() + chassis.drive_sensor_right()) / 2.0;

    double power = kV * des_vel + kA * des_acc + kP * (des_pos - actual);
    power *= dir;

    // Heading hold -- keep heading at 0.  If it curves instead of going
    // straight, flip the sign of kP_h.
    double turn = kP_h * (0.0 - chassis.drive_imu_get());

    chassis.drive_set(clamp127(power + turn), clamp127(power - turn));
    pros::delay(ez::util::DELAY_TIME);
    t += dt;
  }

  chassis.drive_set(0, 0);
  chassis.pid_targets_reset();  // hand a clean slate back to EZ-Template's PID
}
