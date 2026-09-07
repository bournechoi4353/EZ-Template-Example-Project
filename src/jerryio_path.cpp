#include "jerryio_path.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>

#include "main.h"

namespace {
// skip spaces/tabs and a comma
const char* skip_sep(const char* p) {
  while (*p == ' ' || *p == '\t' || *p == ',') p++;
  return p;
}

// trapezoid the waypoint speeds along the path's arc length: ramp up over the
// first `ramp` inches, cruise, ramp down at the end. this is what
// MOTION_PROFILE mode does, pure pursuit still steers, the speed just eases
void apply_trapezoid_speeds(std::vector<ez::odom>& pts, int max_speed) {
  if (pts.size() < 2) return;

  // never drop below this mid-path so it can't stall on the ramps
  const int min_speed = 30;

  // cumulative distance up to each point
  std::vector<double> s(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); i++) {
    double dx = pts[i].target.x - pts[i - 1].target.x;
    double dy = pts[i].target.y - pts[i - 1].target.y;
    s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
  }
  double total = s.back();
  if (total <= 0.0) return;

  // ramp over ~12", or a third of the path if it's short
  double ramp = std::min(total / 3.0, 12.0);
  if (ramp <= 0.0) return;

  for (std::size_t i = 0; i < pts.size(); i++) {
    double v = max_speed;
    if (s[i] < ramp)
      v = max_speed * (s[i] / ramp);                 // ramp up
    else if (s[i] > total - ramp)
      v = max_speed * ((total - s[i]) / ramp);       // ramp down
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

  // embedded bytes aren't null terminated, bound the string with size
  std::string text(reinterpret_cast<const char*>(path.buf), path.size);

  // the export doesn't record its unit (uol stays 1 even for cm), so we always
  // treat the file as centimeters. ALWAYS export in cm.
  const double CM_PER_IN = 2.54;

  struct raw_point {
    double x, y, speed;
  };
  std::vector<raw_point> raws;
  double max_file_speed = 0.0;

  std::stringstream ss(text);
  std::string line;
  while (std::getline(ss, line)) {
    // windows line endings from the web export
    if (!line.empty() && line.back() == '\r') line.pop_back();

    std::size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) continue;  // blank
    std::string trimmed = line.substr(start);

    // everything after "endData" is editor metadata
    if (trimmed.rfind("endData", 0) == 0) break;

    // parse "x, y, speed", speed is optional
    const char* c = trimmed.c_str();
    char* next = nullptr;

    double x = std::strtod(c, &next);
    if (next == c) continue;  // header line, skip
    c = skip_sep(next);

    double y = std::strtod(c, &next);
    if (next == c) continue;  // malformed, skip
    c = skip_sep(next);

    double speed = std::strtod(c, &next);
    if (next == c) speed = max_speed;  // no speed column

    raws.push_back({x, y, speed});
    if (speed > max_file_speed) max_file_speed = speed;
  }

  out.reserve(raws.size());
  for (const raw_point& r : raws) {
    ez::odom point;
    point.target.x = r.x / CM_PER_IN;  // cm -> in
    point.target.y = r.y / CM_PER_IN;
    point.target.theta = ez::ANGLE_NOT_SET;  // plain pure-pursuit point, no boomerang
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

  // rotate + shift so the path starts at the origin heading forward. then you
  // just place the robot facing forward and seed odom (0,0,0), no measuring
  // the path's real start angle
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
    printf("[jerryio_path] WARNING: parsed only %d point(s), check that the "
           "path was exported from path.jerryio as x, y, speed text.\n",
           static_cast<int>(points.size()));
    return;
  }

  // end-to-start, still nose-first (retrace it home)
  if (reverse_order) std::reverse(points.begin(), points.end());

  if (how == jerryio::method::MOTION_PROFILE) {
    // ease the speed along the curve; kill EZ's slew so it doesn't double-ramp
    apply_trapezoid_speeds(points, max_speed);
    slew_on = false;
  }

  // injected (not smoothed) pp, jerryio paths are already dense and smooth,
  // no point smoothing twice
  chassis.pid_odom_injected_pp_set(points, slew_on);
}

void jerryio::follow_path_reverse_tail(const jerryio::asset& path, int max_speed) {
  // start_forward, so seed (0,0,0) and place the robot facing forward
  std::vector<ez::odom> pts = path_to_odom(path, max_speed, false, false, true);
  if (pts.size() < 3) {
    printf("[jerryio_path] reverse_tail: need >= 3 points, got %d.\n",
           static_cast<int>(pts.size()));
    return;
  }

  // find the turnaround, where the path doubles back on itself
  std::size_t split = 0;
  for (std::size_t i = 1; i + 1 < pts.size(); i++) {
    double h1 = std::atan2(pts[i].target.x - pts[i - 1].target.x,
                           pts[i].target.y - pts[i - 1].target.y);
    double h2 = std::atan2(pts[i + 1].target.x - pts[i].target.x,
                           pts[i + 1].target.y - pts[i].target.y);
    double d = h2 - h1;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    if (std::fabs(d) > 2.36) {  // heading flipped ~135+ deg
      split = i;
      break;
    }
  }

  printf("[reverse_tail] %d pts, split (turnaround) at %d\n",
         static_cast<int>(pts.size()), static_cast<int>(split));

  if (split == 0) {  // never doubles back, just drive it
    chassis.pid_odom_injected_pp_set(pts, true);
    return;
  }

  // forward to the turnaround
  std::vector<ez::odom> fwd(pts.begin(), pts.begin() + split + 1);
  chassis.pid_odom_injected_pp_set(fwd, true);
  chassis.pid_wait();

  // reverse the rest, backs the output into the goal
  std::vector<ez::odom> rev(pts.begin() + split, pts.end());
  for (auto& p : rev) p.drive_direction = ez::rev;
  chassis.pid_odom_injected_pp_set(rev, false);
  // caller pid_wait()s
}

void jerryio::seed_start_pose(const jerryio::asset& path) {
  std::vector<ez::odom> points = path_to_odom(path, 1, false, false);
  if (points.size() < 2) {
    printf("[jerryio_path] seed_start_pose: need >=2 points, got %d.\n",
           static_cast<int>(points.size()));
    return;
  }

  double x = points[0].target.x;
  double y = points[0].target.y;

  // heading toward the second point. EZ is 0 = +Y, clockwise, so atan2(dx, dy)
  double dx = points[1].target.x - x;
  double dy = points[1].target.y - y;
  double theta = ez::util::to_deg(std::atan2(dx, dy));

  chassis.odom_pose_set({x, y, theta});
  printf("[jerryio_path] start pose -> x: %.2f  y: %.2f  theta: %.2f\n", x, y,
         theta);
}
