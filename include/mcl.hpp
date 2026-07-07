#pragma once

#include "api.h"

/**
 * Monte Carlo Localization (particle filter) for Override -- v1.
 *
 * THE INVARIANT (safety-critical, do not weaken):
 *   "IMU owns heading; MCL owns position; a clean wall hit may RAISE AN ALARM
 *    about heading but never silently change it."
 *
 * Why: the Override field is 4-fold symmetric, so a range-only filter CANNOT
 * tell the four 90-degree rotations apart -- heading must come from the IMU,
 * and MCL is given ZERO authority over it.  That's enforced structurally:
 * particles are 2-DOF (x, y) and all share the live IMU heading, so there is
 * no heading state to corrupt.  Corrections only ever call odom_x_set /
 * odom_y_set -- never theta.
 *
 * What to expect: this is "automatic wall-reset that can also work off goals
 * (v2)", NOT continuous cm-accurate tracking.  With 56 cups + 63 pins +
 * robots in play, most beams hit moving junk; MCL corrects OPPORTUNISTICALLY
 * when it gets clean wall readings and coasts on odometry the rest of the
 * time.  Don't tune it toward continuous tracking -- that destabilizes it.
 *
 * Correction safety: corrections are QUEUED, and flushed only when the robot
 * is settled (between motions).  Never mid-servo -- a pose jump re-aims pure
 * pursuit / spikes the PID D-term and the lurch is worse than the drift.
 * X and Y gate independently (near one wall only one axis is observable).
 *
 * Usage (auton):
 *   chassis.odom_xyt_set(START_X, START_Y, START_THETA);  // field coords!
 *   mcl::start(START_X, START_Y);          // seed cloud at the known start
 *   ... normal EZ motions ...
 *   chassis.pid_wait();
 *   mcl::flush_if_safe();                  // natural safe point
 * Auto-flush (on by default) also applies pending corrections whenever the
 * robot has been still for a few ticks, so explicit calls are optional.
 */
namespace mcl {

/** One distance sensor, parameterized -- REPLACE offsets when the robot is
 *  built (placeholders assume 6.3" center->face, like wall_reset).
 *  Offsets are in the robot BODY frame: +x = right, +y = forward, inches.
 *  facing_deg is body-relative, CW: 0 = forward, 90 = right, 180 = back.
 *  beam_height documents which map layer the sensor sees (keep < ~4" so it
 *  ranges wall/goal bases, not cup/pin stacks or robots) -- not used in the
 *  2D math. */
struct sensor_cfg {
  pros::Distance* dev;
  double off_x, off_y;
  double facing_deg;
  double beam_height;
};

/** Replace the default 3-sensor table (back/right/left placeholders). Call
 *  before start(). */
void sensors_set(const std::vector<sensor_cfg>& sensors);

/** Seed the cloud at a KNOWN start pose (field coords, inches) and start the
 *  background task (30 Hz).  spread = 1-sigma of the seed cloud.  No global
 *  localization -- on this symmetric field it cannot work (see header). */
void start(double x, double y, double seed_spread = 2.0);

/** Pause the filter (task idles; state kept). */
void stop();

/** Enable/disable automatic correction flushing when settled (default ON). */
void auto_flush_set(bool on);

/** Apply the pending correction to EZ odom IF safe: robot settled + per-axis
 *  confidence gates pass.  Returns true if anything was applied.  Call after
 *  pid_wait() at natural motion boundaries (also fires automatically if
 *  auto-flush is on). NEVER touches heading. */
bool flush_if_safe();

// --- Telemetry (for the brain screen / test auton) ---
double x();                //!< current estimate (field inches)
double y();
double spread_x();         //!< 1-sigma cloud spread per axis -- confidence
double spread_y();
double pending_x();        //!< queued correction (estimate - odom), inches
double pending_y();
bool confident_x();        //!< per-axis: spread low + enough recent good hits
bool confident_y();
int sensors_accepted();    //!< beams used in the last update
int sensors_gated();       //!< beams rejected by the innovation gate
int flush_count();         //!< corrections applied since start()

}  // namespace mcl
