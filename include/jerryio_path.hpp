#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "EZ-Template/api.hpp"

/**
 * path.jerryio path support for EZ-Template.
 *
 * This lets you design paths visually on https://path.jerryio.com, export the
 * .txt file, and run it here using EZ-Template's own pure-pursuit follower -- no
 * SD card required.  The path is compiled directly into the program.
 *
 * This reads path.jerryio's normal export (the file that starts with
 * "#PATH-POINTS-START" and has "x,y,speed" lines).  It also accepts the older
 * "x, y, speed ... endData" style.  Either way, just export and drop it in.
 *
 * --- WORKFLOW (adding a new path) -------------------------------------------
 *   1. Design the path on path.jerryio, export it.
 *   2. Drop the .txt file into the  static/  folder (e.g. static/red_left.txt).
 *   3. Add one line to  src/path_assets.S  to embed it:
 *          PATH_ASSET red_left
 *      If the file name has dots (e.g. "path.jerryio.txt"), give a symbol-safe
 *      name plus the real file name:
 *          PATH_ASSET path_jerryio, "path.jerryio.txt"
 *   4. In your auton file, declare it once and follow it:
 *          ASSET(red_left_txt);
 *          ...
 *          jerryio::seed_start_pose(red_left_txt);   // optional, see below
 *          jerryio::follow_path(red_left_txt, DRIVE_SPEED);
 *          chassis.pid_wait();
 *   5. Run `pros make clean` once after adding/changing a .txt (the build can't
 *      auto-detect that the embedded file changed).
 *
 * --- COORDINATES ------------------------------------------------------------
 * path.jerryio uses the VEX GPS frame: origin at the FIELD CENTER, +X east
 * (right), +Y north (forward), heading 0 = +Y and increasing clockwise -- the
 * same frame EZ-Template odom uses.  ALWAYS export from path.jerryio in
 * CENTIMETERS (its default).  The text export doesn't record the unit, so this
 * code assumes cm and converts to inches (/ 2.54) -- exporting in inches would
 * come out 2.54x too small.
 *
 * Because the path is in absolute field coordinates, odom must know where the
 * robot really is before you follow.  The easy way is seed_start_pose(), which
 * snaps odom to the path's first point -- valid only if the robot is physically
 * placed there.  Always sanity-check x/y/theta on the brain before trusting it.
 */

namespace jerryio {

/**
 * A path file that has been compiled into the program binary.
 *
 * Created for you by the ASSET() macro -- you normally won't build one of these
 * by hand.  `buf` points at the embedded file bytes, `size` is its length.
 */
struct asset {
  const uint8_t* buf;
  std::size_t size;
};

/**
 * How to follow a path.
 *
 * Both follow the full 2D curve using EZ-Template's pure-pursuit steering --
 * the only difference is how speed is handled along the path:
 *
 *   PURE_PURSUIT   - holds a flat max speed the whole way (EZ slews at the
 *                    start, PID eases out at the end).  FASTEST -- best when
 *                    you need to cover ground in the 15s autonomous.
 *
 *   MOTION_PROFILE - shapes speed with a trapezoidal profile over the path's
 *                    length (ramp up -> cruise -> ramp down).  SMOOTHEST --
 *                    gentle starts/stops, best for a tall/tippy load or when
 *                    you need a consistent, repeatable finish.
 *
 * Note: the standalone profiled_drive() in motion_profile.cpp is a STRAIGHT-
 * line-only profile and cannot follow a curve, so MOTION_PROFILE here applies
 * the same trapezoidal idea *along the curve* instead of driving straight.
 */
enum class method {
  PURE_PURSUIT,
  MOTION_PROFILE
};

/**
 * Declares a compiled-in path asset by name.
 *
 * For `ASSET(red_left_txt);`, the matching file is  static/red_left.txt  and it
 * must be embedded by a `PATH_ASSET red_left` line in src/path_assets.S.
 *
 * \param x
 *        the asset name: the file name (without the .txt) followed by `_txt`.
 */
#define ASSET(x)                                          \
  extern "C" const uint8_t x##_start[];                   \
  extern "C" const uint8_t x##_end[];                     \
  static const jerryio::asset x = {                       \
      x##_start, static_cast<std::size_t>(x##_end - x##_start)}

/**
 * Parses a path.jerryio path into EZ-Template odom waypoints.
 *
 * Lines are read as "x, y, speed" until an "endData" line (everything after it,
 * the editor metadata, is ignored).  Non-numeric lines are skipped, so headers
 * are tolerated.  Each waypoint's heading is left unset, so EZ-Template treats
 * them as pure-pursuit points (no boomerang).
 *
 * \param path
 *        the compiled-in path (from ASSET()).
 * \param max_speed
 *        the maximum drive speed, out of 127.
 * \param reverse
 *        true to drive the path backwards (same point order, robot reversed).
 * \param scale_speeds
 *        true to slow down on tight sections using the per-point speeds baked
 *        into the file (scaled so the file's fastest point == max_speed).
 *        false (default) uses max_speed for the whole path -- more predictable.
 */
std::vector<ez::odom> path_to_odom(const asset& path, int max_speed,
                                   bool reverse = false,
                                   bool scale_speeds = false,
                                   bool start_forward = false);

/**
 * Follows a path.jerryio path using EZ-Template's pure pursuit.
 *
 * This only *starts* the motion (like chassis.pid_odom_set) -- call
 * chassis.pid_wait() / pid_wait_until() yourself afterwards so you can run
 * subsystems mid-path or chain motions, just like any other EZ-Template motion.
 *
 * \param path
 *        the compiled-in path (from ASSET()).
 * \param max_speed
 *        the maximum drive speed, out of 127.
 * \param how
 *        PURE_PURSUIT (flat speed, fastest) or MOTION_PROFILE (eased speed,
 *        smoothest).  See the `method` enum.  Defaults to PURE_PURSUIT.
 * \param reverse
 *        true to drive the path backwards.
 * \param slew_on
 *        true to slew (ramp up) at the start.  Ignored for MOTION_PROFILE,
 *        which does its own ramp-up.
 * \param reverse_order
 *        true to drive the path from its LAST point back to its FIRST (still
 *        nose-first).  Handy for retracing a path back to where it started.
 *        This is different from `reverse`, which drives the robot backwards.
 * \param start_forward
 *        true rotates the whole path so it STARTS heading forward (+Y) from the
 *        origin -- place the robot FACING FORWARD and seed odom to (0,0,0); it
 *        drives straight into the path.  No need to match the path's start angle.
 */
void follow_path(const asset& path, int max_speed,
                 method how = method::PURE_PURSUIT, bool reverse = false,
                 bool slew_on = true, bool reverse_order = false,
                 bool start_forward = false);

/**
 * For paths that DOUBLE BACK on themselves (drive out, then return): follows the
 * path FORWARD to the turnaround point, then drives the rest in REVERSE -- e.g.
 * collect on the way out, then back the output into the goal on the way home.
 *
 * Uses start_forward, so seed odom to (0,0,0) and place the robot FACING FORWARD.
 * It pid_wait()s internally between the forward and reverse halves; call
 * pid_wait() yourself after it returns to wait for the reverse half to finish.
 */
void follow_path_reverse_tail(const asset& path, int max_speed);

/**
 * Snaps odometry to the path's starting point, facing along the path.
 *
 * Use this right before follow_path() when the robot is physically placed at
 * the path's first point.  It sets odom x/y to that point (in EZ inches) and
 * the heading toward the second point.  No-op if the path has fewer than 2
 * points.  ALWAYS verify the resulting x/y/theta on the brain screen.
 *
 * \param path
 *        the compiled-in path (from ASSET()).
 */
void seed_start_pose(const asset& path);

}  // namespace jerryio
