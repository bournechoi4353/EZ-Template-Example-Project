#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "EZ-Template/api.hpp"

// path.jerryio support: design a path on https://path.jerryio.com, export
// the txt, and run it through EZ-Template's pure pursuit. the file gets
// compiled into the binary, no SD card needed.
//
// adding a path:
//   1. export from path.jerryio in CENTIMETERS (see below), drop it in static/
//   2. add `PATH_ASSET <name>` to src/path_assets.S (2-arg form if the file
//      name has dots)
//   3. declare ASSET(<name>_txt) in your auton file and follow it
//   4. `pros make clean`, the build can't tell a .txt changed
//
// coordinates: jerryio uses the same frame as EZ odom (origin at field
// center, +x right, +y forward, heading 0 = +y, clockwise). the export
// doesn't record what unit it's in, so this code assumes CENTIMETERS
// (jerryio's default) and divides by 2.54. export in inches and everything
// comes out 2.54x too small.
//
// the path is in absolute field coords, so odom has to know where the robot
// actually is before following, seed it yourself or use seed_start_pose().

namespace jerryio {

// an embedded path file. the ASSET() macro builds these for you
struct asset {
  const uint8_t* buf;
  std::size_t size;
};

// how speed is handled along the path (steering is pure pursuit either way):
//   PURE_PURSUIT   - flat max speed the whole way. fastest, use it when you
//                    need to cover ground in the 15s auton
//   MOTION_PROFILE - trapezoid speed along the path (ease in, cruise, ease
//                    out). smoothest, tippy loads, repeatable stops
enum class method {
  PURE_PURSUIT,
  MOTION_PROFILE
};

// declare an embedded path by name: ASSET(red_left_txt) matches
// static/red_left.txt embedded by a `PATH_ASSET red_left` line in the .S
#define ASSET(x)                                          \
  extern "C" const uint8_t x##_start[];                   \
  extern "C" const uint8_t x##_end[];                     \
  static const jerryio::asset x = {                       \
      x##_start, static_cast<std::size_t>(x##_end - x##_start)}

// parse a jerryio export into EZ odom waypoints. reads "x, y, speed" lines,
// skips headers, stops at "endData". headings stay unset so EZ treats them
// as plain pure-pursuit points. scale_speeds=true uses the per-point speeds
// baked into the file (scaled so its fastest point == max_speed); false
// just runs max_speed everywhere, which is more predictable
std::vector<ez::odom> path_to_odom(const asset& path, int max_speed,
                                   bool reverse = false,
                                   bool scale_speeds = false,
                                   bool start_forward = false);

// follow a path. this only STARTS the motion (like pid_odom_set), call
// chassis.pid_wait() yourself so you can run subsystems mid-path.
//   reverse       = drive the path backwards
//   slew_on       = ramp up at the start (ignored for MOTION_PROFILE, it
//                   does its own ramp)
//   reverse_order = run last point to first, still nose-first (retrace home)
//   start_forward = rotate the whole path so it starts heading forward from
//                   the origin. place the robot facing forward, seed odom
//                   (0,0,0), no measuring the start angle
void follow_path(const asset& path, int max_speed,
                 method how = method::PURE_PURSUIT, bool reverse = false,
                 bool slew_on = true, bool reverse_order = false,
                 bool start_forward = false);

// for paths that double back on themselves: drives forward to the turnaround,
// then runs the rest in reverse, e.g. collect on the way out, back the
// output into the goal on the way home. uses start_forward, so seed (0,0,0)
// and place the robot facing forward. it pid_wait()s between the halves;
// pid_wait() yourself after it returns for the reverse half
void follow_path_reverse_tail(const asset& path, int max_speed);

// snap odom to the path's first point, facing along the path. only valid if
// the robot is physically sitting there. check the brain screen after
void seed_start_pose(const asset& path);

}  // namespace jerryio
