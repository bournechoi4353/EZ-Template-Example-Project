#pragma once

#include "api.h"

// monte carlo localization (particle filter), v1.
//
// the one rule, do not weaken it: IMU owns heading, MCL owns position. a
// clean wall hit may raise an alarm about heading but never silently change
// it. the field is 4-fold symmetric, so a range-only filter literally can't
// tell the four 90-degree rotations apart, heading HAS to come from the
// IMU. that's enforced structurally: particles are (x, y) only and all share
// the live IMU heading, so there's no heading state to corrupt. corrections
// only ever call odom_x_set/odom_y_set, never theta.
//
// what to expect: this is an automatic wall reset, not continuous
// cm-accurate tracking. with all the cups/pins/robots out there most beams
// hit moving junk, it corrects when it gets clean wall readings and coasts
// on odom the rest of the time. don't tune it toward continuous tracking,
// that destabilizes it.
//
// corrections queue up and only flush when the robot is settled, between
// motions. a pose jump mid-drive re-aims pure pursuit and spikes the PID,
// and the lurch is worse than the drift. x and y gate independently, since near
// one wall only one axis is observable.
//
// usage in an auton:
//   chassis.odom_xyt_set(START_X, START_Y, START_THETA);  // field coords!
//   mcl::start(START_X, START_Y);
//   ... normal EZ motions ...
//   chassis.pid_wait();
//   mcl::flush_if_safe();
// auto-flush (on by default) also fires whenever the robot sits still for a
// bit, so the explicit calls are optional.
namespace mcl {

// one distance sensor. offsets are body frame (+x right, +y forward, inches),
// facing_deg is clockwise from forward (90 right, 180 back, 270 left).
// replace the placeholder offsets once the robot is built. beam_height isn't
// used in the math (map is 2D), it's a note, keep sensors under ~4" so
// they range walls, not cup stacks and robots
struct sensor_cfg {
  pros::Distance* dev;
  double off_x, off_y;
  double facing_deg;
  double beam_height;
};

// replace the default back/right/left sensor table. call before start()
void sensors_set(const std::vector<sensor_cfg>& sensors);

// seed the cloud at a KNOWN pose (field inches) and start the 30 Hz task.
// no global localization, can't work on a symmetric field (see above)
void start(double x, double y, double seed_spread = 2.0);

// pause the filter (state kept)
void stop();

// enable/disable auto-flush when settled (default on)
void auto_flush_set(bool on);

// apply the pending correction if it's safe: robot settled + per-axis
// confidence gates pass. returns true if anything was applied. call after
// pid_wait() at natural boundaries. never touches heading
bool flush_if_safe();

// telemetry for the brain screen / test auton
double x();              // current estimate, field inches
double y();
double spread_x();       // 1-sigma cloud spread per axis
double spread_y();
double pending_x();      // queued correction (estimate - odom), inches
double pending_y();
bool confident_x();      // spread low + enough recent good beams
bool confident_y();
int sensors_accepted();  // beams used in the last update
int sensors_gated();     // beams rejected by the innovation gate
int flush_count();       // corrections applied since start()

}  // namespace mcl
