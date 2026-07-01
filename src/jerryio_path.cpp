#include "jerryio_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

#include "main.h"

namespace {
// Advance past any spaces, tabs, and a single field separator (comma).
const char* skip_sep(const char* p) {
  while (*p == ' ' || *p == '\t' || *p == ',') p++;
  return p;
}

// Shape each waypoint's speed with a trapezoidal profile along the PATH'S
// ARC LENGTH: ramp up over the first `ramp` inches, cruise at max_speed, then
// ramp down over the last `ramp` inches.  This is the MOTION_PROFILE behavior --
// pure pursuit still steers along the curve, but the speed eases in and out.
void apply_trapezoid_speeds(std::vector<ez::odom>& pts, int max_speed) {
  if (pts.size() < 2) return;

  // Speed never drops below this mid-path, so it can't stall on the ramps.
  const int min_speed = 30;

  // Cumulative distance traveled up to each point.
  std::vector<double> s(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); i++) {
    double dx = pts[i].target.x - pts[i - 1].target.x;
    double dy = pts[i].target.y - pts[i - 1].target.y;
    s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }
  double total = s.back();
  if (total <= 0.0) return;

  // Ramp over ~12 in (or 1/3 of the path on short paths, leaving a cruise).
  double ramp = std::min(total / 3.0, 12.0);
  if (ramp <= 0.0) return;

  for (std::size_t i = 0; i < pts.size(); i++) {
    double v = max_speed;
    if (s[i] < ramp)
      v = max_speed * (s[i] / ramp);                 // ramping up
    else if (s[i] > total - ramp)
      v = max_speed * ((total - s[i]) / ramp);       // ramping down
    int iv = static_cast<int>(v);
    if (iv < min_speed) iv = min_speed;
    if (iv > max_speed) iv = max_speed;
    pts[i].max_xy_speed = iv;
  }
}
}  // namespace

std::vector<ez::odom> jerryio::path_to_odom(const jerryio::asset& path,
                                            int max_speed, bool reverse,
                                            bool scale_speeds, bool start_forward) {
  std::vector<ez::odom> out;
  if (path.buf == nullptr || path.size == 0) return out;

  // The embedded bytes are the raw .txt contents (not guaranteed null
  // terminated), so build a bounded string from buf + size.
  std::string text(reinterpret_cast<const char*>(path.buf), path.size);

  // path.jerryio's default export unit is CENTIMETERS, and the text export does
  // NOT record which unit was used (the "uol" field stays 1 even for cm).  So we
  // always treat the file as centimeters and convert to EZ-Template's inches.
  // ==> ALWAYS export your paths in CENTIMETERS. <==
  const double CM_PER_IN = 2.54;

  struct raw_point {
    double x, y, speed;
  };
  std::vector<raw_point> raws;
  double max_file_speed = 0.0;

  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    // Tolerate Windows line endings from the web export.
    if (!line.empty() && line.back() == '\r') line.pop_back();

    // Trim leading whitespace.
    std::size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) continue;  // blank line
    std::string trimmed = line.substr(start);

    // path.jerryio writes the editor metadata after an "endData" marker.
    if (trimmed.rfind("endData", 0) == 0) break;

    // Parse "x, y, speed".  speed is optional and defaults to max_speed.
    const char* c = trimmed.c_str();
    char* next = nullptr;

    double x = std::strtod(c, &next);
    if (next == c) continue;  // not a coordinate line (header) -> skip
    c = skip_sep(next);

    double y = std::strtod(c, &next);
    if (next == c) continue;  // malformed -> skip
    c = skip_sep(next);

    double speed = std::strtod(c, &next);
    if (next == c) speed = max_speed;  // no speed column on this line

    raws.push_back({x, y, speed});
    if (speed > max_file_speed) max_file_speed = speed;
  }

  out.reserve(raws.size());
  for (const raw_point& r : raws) {
    ez::odom point;
    point.target.x = r.x / CM_PER_IN;  // cm -> EZ inches
    point.target.y = r.y / CM_PER_IN;
    point.target.theta = ez::ANGLE_NOT_SET;  // pure-pursuit point (no boomerang)
    point.drive_direction = reverse ? ez::rev : ez::fwd;

    int speed = max_speed;
    if (scale_speeds && max_file_speed > 0.0) {
      speed = static_cast<int>(
          ez::util::clamp(max_speed * (r.speed / max_file_speed),
                          static_cast<double>(max_speed), 0.0));
    }
    point.max_xy_speed = speed;

    out.push_back(point);
  }

  // Rotate + translate so the path STARTS at the origin heading forward (+Y).
  // Then you just place the robot FACING FORWARD at the start (no need to match
  // the path's real start angle) and it drives straight into the path.  This
  // makes the path relative to the robot's start, so seed odom to (0,0,0).
  if (start_forward && out.size() >= 2) {
    double x0 = out[0].target.x, y0 = out[0].target.y;
    double th = std::atan2(out[1].target.x - x0, out[1].target.y - y0);  // initial heading
    double c = std::cos(th), s = std::sin(th);
    for (auto& p : out) {
      double ex = p.target.x - x0, ny = p.target.y - y0;
      p.target.x = ex * c - ny * s;
      p.target.y = ex * s + ny * c;
    }
  }

  return out;
}

void jerryio::follow_path(const jerryio::asset& path, int max_speed,
                          jerryio::method how, bool reverse, bool slew_on,
                          bool reverse_order, bool start_forward) {
  std::vector<ez::odom> points = path_to_odom(path, max_speed, reverse, false, start_forward);

  if (points.size() < 2) {
    printf("[jerryio_path] WARNING: parsed only %d point(s) -- check that the "
           "path was exported from path.jerryio as x, y, speed text.\n",
           static_cast<int>(points.size()));
    return;
  }

  // Drive the path end-to-start (still nose-first) -- e.g. to retrace it home.
  if (reverse_order) std::reverse(points.begin(), points.end());

  if (how == jerryio::method::MOTION_PROFILE) {
    // Ease the speed up and down along the curve, then let pure pursuit steer.
    // We turn OFF EZ's slew here so it doesn't double-ramp the start.
    apply_trapezoid_speeds(points, max_speed);
    slew_on = false;
  }

  // EZ-Template's injected pure pursuit: it spaces the points out evenly and
  // follows them.  We use the *injected* (not smoothed) variant because
  // path.jerryio already gives a dense, smooth path -- no need to smooth twice.
  chassis.pid_odom_injected_pp_set(points, slew_on);
}

void jerryio::follow_path_reverse_tail(const jerryio::asset& path, int max_speed) {
  // start_forward = true, so seed odom to (0,0,0) and place the robot FORWARD.
  std::vector<ez::odom> pts = path_to_odom(path, max_speed, false, false, true);
  if (pts.size() < 3) {
    printf("[jerryio_path] reverse_tail: need >= 3 points, got %d.\n",
           static_cast<int>(pts.size()));
    return;
  }

  // Find the TURNAROUND: the point where the path doubles back (heading flips
  // ~180 deg) -- e.g. it drives to a wall, then comes back out.
  std::size_t split = 0;
  for (std::size_t i = 1; i + 1 < pts.size(); i++) {
    double h1 = std::atan2(pts[i].target.x - pts[i - 1].target.x,
                           pts[i].target.y - pts[i - 1].target.y);
    double h2 = std::atan2(pts[i + 1].target.x - pts[i].target.x,
                           pts[i + 1].target.y - pts[i].target.y);
    double d = h2 - h1;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    if (std::fabs(d) > 2.36) {  // ~135 deg -> it's doubling back
      split = i;
      break;
    }
  }

  printf("[reverse_tail] %d pts, split (turnaround) at %d\n",
         static_cast<int>(pts.size()), static_cast<int>(split));

  if (split == 0) {  // no turnaround found -- just drive it all forward
    chassis.pid_odom_injected_pp_set(pts, true);
    return;
  }

  // 1) FORWARD to the turnaround.
  std::vector<ez::odom> fwd(pts.begin(), pts.begin() + split + 1);
  chassis.pid_odom_injected_pp_set(fwd, true);
  chassis.pid_wait();

  // 2) REVERSE the rest -- back the output into the goal.
  std::vector<ez::odom> rev(pts.begin() + split, pts.end());
  for (auto& p : rev) p.drive_direction = ez::rev;
  chassis.pid_odom_injected_pp_set(rev, false);
  // caller calls pid_wait() afterwards
}

void jerryio::seed_start_pose(const jerryio::asset& path) {
  // path_to_odom already converts to EZ inches.
  std::vector<ez::odom> points = path_to_odom(path, 1, false, false);
  if (points.size() < 2) {
    printf("[jerryio_path] seed_start_pose: need >=2 points, got %d.\n",
           static_cast<int>(points.size()));
    return;
  }

  double x = points[0].target.x;
  double y = points[0].target.y;

  // Heading toward the next point.  EZ uses 0 = +Y (north), clockwise positive,
  // which is atan2(east, north) = atan2(dx, dy).
  double dx = points[1].target.x - x;
  double dy = points[1].target.y - y;
  double theta = ez::util::to_deg(std::atan2(dx, dy));

  chassis.odom_pose_set({x, y, theta});
  printf("[jerryio_path] start pose -> x: %.2f  y: %.2f  theta: %.2f\n", x, y,
         theta);
}
