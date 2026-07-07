#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "field_map.hpp"
#include "main.h"

/**
 * Monte Carlo Localization engine -- see mcl.hpp for the contract.
 *
 * "IMU owns heading; MCL owns position; a clean wall hit may RAISE AN ALARM
 *  about heading but never silently change it."
 *
 * Design decisions that are NOT accidental (don't "simplify" them away):
 *  - Particles are 2-DOF (x, y).  Heading comes from the IMU every tick and
 *    is shared by all particles, so MCL structurally CANNOT edit heading.
 *    Heading uncertainty still exists physically, so ~1 deg of it is folded
 *    into the beam sigma instead (1 deg at 36" ~= 0.6" of range error).
 *  - Robust beam model: Gaussian around the predicted range with a FLAT
 *    OUTLIER FLOOR, so one blocked beam (cup/pin/robot) can't zero a good
 *    particle's weight.
 *  - ADAPTIVE innovation gate: readings that disagree with the current
 *    estimate get dropped -- but the gate widens with cloud spread, so after
 *    a collision (spread blown out) the gate stays open and the exact wall
 *    readings needed to re-converge are NOT rejected.  A fixed gate would
 *    fight its own recovery.
 *  - Max-range trust: short readings are suspect by default (that's where
 *    cups/pins/robots live); readings that agree with a far predicted wall
 *    carry full authority.
 *  - Process noise scales with distance traveled and is ZERO at rest -- the
 *    cloud must not inflate while the robot sits still.  Diversity at rest
 *    comes from the small regularization jitter injected at RESAMPLE time
 *    (otherwise repeated identical measurements collapse the cloud onto a
 *    handful of duplicated particles = sample impoverishment).
 *  - Corrections are queued and flushed only when settled; X and Y gate
 *    independently (near a single wall only one axis is observable -- the
 *    cloud stays fat in the blind axis and that axis simply doesn't flush).
 */

namespace {

// ---------------- Tunables (retune on the real robot) ----------------
constexpr int N_PARTICLES = 300;   // drop to 150-200 if sharing CPU with AI Vision
constexpr int LOOP_MS = 33;        // ~30 Hz

// Motion model noise
constexpr double K_TRANS = 0.015;      // 1-sigma translational noise, per inch traveled (~1.5%)
constexpr double K_TURN_XY = 0.010;    // extra x/y noise per DEGREE turned (tracker wobble)
constexpr double RESAMPLE_JITTER = 0.15;  // regularization jitter (in) injected at resample

// Beam model
constexpr double SIGMA_BASE = 1.5;       // base range sigma (in)
constexpr double SIGMA_HEADING = 0.0175; // ~1 deg of shared-heading error, folded in as rad*range
constexpr double OUTLIER_FLOOR = 0.08;   // flat floor (relative to Gaussian peak 1.0)
constexpr double GATE_SIGMAS = 3.0;      // innovation gate width, in sigmas
constexpr double GATE_PAD = 1.0;         // + fixed pad (in)

// Sensor validity (mirrors wall_reset's proven gates)
constexpr int CONF_MIN = 30;         // pros::Distance confidence 0-63
constexpr double RANGE_MIN = 2.0;    // inches
constexpr double RANGE_MAX = 36.0;   // trusted max range (same as wall_reset)

// Max-range trust: authority ramps 0.5 -> 1.0 between these readings (in)
constexpr double AUTH_LOW = 8.0;
constexpr double AUTH_FULL = 24.0;

// Correction gates
constexpr double CONF_SPREAD = 3.0;   // per-axis: cloud 1-sigma must be under this (in)
constexpr int GOOD_HITS_MIN = 5;      // ...and >= this many updates w/ accepted beams in the last window
constexpr int GOOD_HITS_WINDOW = 15;  // window length (updates)
constexpr double CORR_MIN = 0.25;     // ignore corrections smaller than this (in)
constexpr double CORR_MAX = 8.0;      // clamp per flush; larger pending = alarm, investigate
constexpr int STILL_TICKS = 5;        // settled = this many consecutive still ticks (~165 ms)
constexpr double STILL_POS = 0.05;    // "still" thresholds per tick
constexpr double STILL_THETA = 0.2;   // degrees

constexpr double DEG2RAD = M_PI / 180.0;

// ---------------- State ----------------
struct particle {
  double x, y, w;
};

std::vector<particle> parts;
std::vector<mcl::sensor_cfg> sensors;  // filled with defaults on first start()

pros::Mutex mtx;          // guards everything below (tick vs flush_if_safe race)
pros::Task* task = nullptr;
bool running = false;
bool auto_flush = true;

double last_ox = 0, last_oy = 0, last_th = 0;  // odom at previous tick
double est_x = 0, est_y = 0;                   // weighted-mean estimate
double sprd_x = 99, sprd_y = 99;               // per-axis cloud 1-sigma
int still_ticks = 0;
int accepted_count = 0, gated_count = 0;
bool good_hits[GOOD_HITS_WINDOW] = {};         // ring buffer of "had accepted beams"
int good_idx = 0;
int flushes = 0;

std::mt19937 rng(12345);  // deterministic seed: reproducible runs, and V5 has no
                          // entropy source worth chasing.  Reseeded from micros in start().

double gauss(double sigma) {
  std::normal_distribution<double> d(0.0, sigma);
  return d(rng);
}

int good_hit_count() {
  int n = 0;
  for (bool b : good_hits) n += b ? 1 : 0;
  return n;
}

bool confident_axis(double spread) {
  return spread < CONF_SPREAD && good_hit_count() >= GOOD_HITS_MIN;
}

// One accepted (validity- and innovation-gated) beam, precomputed for the
// per-particle loop.  Heading is SHARED, so the rotated sensor offset and ray
// direction are constants across all particles this tick.
struct beam {
  double reading;      // inches
  double fox, foy;     // sensor face offset, rotated into WORLD frame
  double dirx, diry;   // world-frame unit ray
  double sigma;        // beam sigma (base + heading-uncertainty inflation)
  double auth;         // 0.5..1.0 weight authority (max-range trust)
};

// Recompute estimate + per-axis spread from the (normalized) cloud.
void refresh_estimate() {
  double mx = 0, my = 0;
  for (const particle& p : parts) {
    mx += p.w * p.x;
    my += p.w * p.y;
  }
  double vx = 0, vy = 0;
  for (const particle& p : parts) {
    vx += p.w * (p.x - mx) * (p.x - mx);
    vy += p.w * (p.y - my) * (p.y - my);
  }
  est_x = mx;
  est_y = my;
  sprd_x = std::sqrt(vx);
  sprd_y = std::sqrt(vy);
}

// Low-variance (systematic) resampling + regularization jitter.
void resample() {
  std::vector<particle> next;
  next.reserve(parts.size());
  double step = 1.0 / parts.size();
  std::uniform_real_distribution<double> u(0.0, step);
  double r = u(rng), c = parts[0].w;
  std::size_t i = 0;
  for (std::size_t m = 0; m < parts.size(); m++) {
    double target = r + m * step;
    while (c < target && i + 1 < parts.size()) c += parts[++i].w;
    particle p = parts[i];
    p.x += gauss(RESAMPLE_JITTER);  // keep diversity: see header comment on
    p.y += gauss(RESAMPLE_JITTER);  // sample impoverishment at rest
    p.w = step;
    next.push_back(p);
  }
  parts.swap(next);
}

// Apply the pending correction if the safety gates pass.  Caller holds mtx.
// THE one place odom is ever written -- and it's odom_x_set/odom_y_set ONLY.
bool flush_locked() {
  if (still_ticks < STILL_TICKS) return false;  // never mid-motion

  double ox = chassis.odom_x_get(), oy = chassis.odom_y_get();
  double px = est_x - ox, py = est_y - oy;

  double dx = 0, dy = 0;
  if (confident_axis(sprd_x) && std::fabs(px) >= CORR_MIN)
    dx = std::clamp(px, -CORR_MAX, CORR_MAX);
  if (confident_axis(sprd_y) && std::fabs(py) >= CORR_MIN)
    dy = std::clamp(py, -CORR_MAX, CORR_MAX);
  if (dx == 0.0 && dy == 0.0) return false;

  if (std::fabs(px) > CORR_MAX || std::fabs(py) > CORR_MAX)
    printf("[mcl] ALARM: pending correction (%.1f, %.1f) exceeds %.1f\" -- clamped, investigate\n",
           px, py, CORR_MAX);

  chassis.odom_x_set(ox + dx);
  chassis.odom_y_set(oy + dy);
  // The set jumps odom -- compensate the motion model's reference so next
  // tick's odom delta doesn't ALSO shift the particles by the correction.
  last_ox += dx;
  last_oy += dy;
  flushes++;
  printf("[mcl] correction applied: (%+.2f, %+.2f)\n", dx, dy);
  return true;
}

void tick() {
  mtx.take();

  // ---- 1. Odometry in, deltas out ----
  double ox = chassis.odom_x_get(), oy = chassis.odom_y_get();
  double th = chassis.odom_theta_get();  // degrees, CW+, IMU-backed
  double dx = ox - last_ox, dy = oy - last_oy;
  double dth = th - last_th;
  while (dth > 180) dth -= 360;
  while (dth < -180) dth += 360;
  last_ox = ox;
  last_oy = oy;
  last_th = th;

  double dist = std::hypot(dx, dy);
  bool still = dist < STILL_POS && std::fabs(dth) < STILL_THETA;
  still_ticks = still ? still_ticks + 1 : 0;

  // ---- 2. Motion model: shift cloud by the odom delta (+ noise if moving) ----
  if (!still) {
    double s = K_TRANS * dist + K_TURN_XY * std::fabs(dth);
    for (particle& p : parts) {
      p.x += dx + gauss(s);
      p.y += dy + gauss(s);
    }
  } else {
    for (particle& p : parts) {
      p.x += dx;  // sub-threshold creep still tracks; no noise at rest
      p.y += dy;
    }
  }

  // ---- 3-5. Read sensors, validity gate, innovation gate ----
  double h = th * DEG2RAD;
  double c = std::cos(h), s = std::sin(h);
  std::vector<beam> beams;
  accepted_count = 0;
  gated_count = 0;

  for (const mcl::sensor_cfg& sc : sensors) {
    double reading = sc.dev->get() / 25.4;  // mm -> in
    if (sc.dev->get_confidence() < CONF_MIN) continue;
    if (reading < RANGE_MIN || reading > RANGE_MAX) continue;

    // Body -> world (the ONE trig rule: heading h -> direction (sin h, cos h)).
    beam b;
    b.reading = reading;
    b.fox = sc.off_x * c + sc.off_y * s;
    b.foy = -sc.off_x * s + sc.off_y * c;
    double a = h + sc.facing_deg * DEG2RAD;
    b.dirx = std::sin(a);
    b.diry = std::cos(a);

    // Innovation gate, vs the CURRENT ESTIMATE, widened by cloud spread along
    // the ray -- wide after a collision (recovery stays possible), tight when
    // confident (a cup 12" in front of the wall gets rejected here).
    double pred_est = field::raycast(est_x + b.fox, est_y + b.foy, b.dirx, b.diry);
    b.sigma = SIGMA_BASE + SIGMA_HEADING * reading;
    double spread_ray = std::sqrt(sprd_x * b.dirx * sprd_x * b.dirx +
                                  sprd_y * b.diry * sprd_y * b.diry);
    if (pred_est < 0 ||
        std::fabs(reading - pred_est) >
            GATE_SIGMAS * std::sqrt(b.sigma * b.sigma + spread_ray * spread_ray) + GATE_PAD) {
      gated_count++;
      continue;
    }

    // Max-range trust: short readings (cup territory) get half authority.
    b.auth = 0.5 + 0.5 * std::clamp((reading - AUTH_LOW) / (AUTH_FULL - AUTH_LOW), 0.0, 1.0);
    beams.push_back(b);
  }
  accepted_count = static_cast<int>(beams.size());
  good_hits[good_idx] = !beams.empty();
  good_idx = (good_idx + 1) % GOOD_HITS_WINDOW;

  // ---- 6. Weight, estimate, resample (only if something was measured) ----
  if (!beams.empty()) {
    double wsum = 0;
    for (particle& p : parts) {
      double lw = 1.0;
      for (const beam& b : beams) {
        double pred = field::raycast(p.x + b.fox, p.y + b.foy, b.dirx, b.diry);
        double f;
        if (pred < 0) {
          f = OUTLIER_FLOOR;  // particle outside the field -- let it die
        } else {
          double z = (b.reading - pred) / b.sigma;
          f = std::fmax(std::exp(-0.5 * z * z), OUTLIER_FLOOR);
        }
        lw *= std::pow(f, b.auth);
      }
      p.w *= lw;
      wsum += p.w;
    }

    if (wsum <= 1e-300) {
      // Degenerate (every beam disagreed with every particle) -- reset weights
      // rather than divide by zero; the innovation gate makes this rare.
      for (particle& p : parts) p.w = 1.0 / parts.size();
    } else {
      double neff_den = 0;
      for (particle& p : parts) {
        p.w /= wsum;
        neff_den += p.w * p.w;
      }
      refresh_estimate();
      if (1.0 / neff_den < parts.size() / 2.0) resample();
    }
  } else {
    // Coast: cloud already shifted with odom; estimate just tracks it.
    refresh_estimate();
  }

  // ---- 7-8. Auto-flush at safe points ----
  if (auto_flush) flush_locked();

  mtx.give();
}

void task_fn() {
  while (true) {
    if (running) tick();
    pros::delay(LOOP_MS);
  }
}

}  // namespace

// ---------------- Public API ----------------

void mcl::sensors_set(const std::vector<mcl::sensor_cfg>& s) {
  mtx.take();
  sensors = s;
  mtx.give();
}

void mcl::start(double x, double y, double seed_spread) {
  mtx.take();

  // Measured sensor geometry (body frame: +x right, +y forward, inches;
  // facing CW from forward: 90 right / 180 back / 270 left).  off_y only
  // matters for the BACK sensor (it aims -Y, so its 5" setback biases range);
  // the side sensors' range depends only on off_x, so their off_y stays 0.
  // beam_height is documentation only (the map is 2D) -- but 9-10" is HIGH and
  // may shoot over a short field wall, which shows up as gated side beams.
  if (sensors.empty()) {
    sensors = {
        {&distanceBack,   3.0, -5.0, 180.0,  5.5},
        {&distanceRight,  6.0,  0.0,  90.0, 10.0},
        {&distanceLeft,  -3.0,  0.0, 270.0,  9.0},
    };
  }

  rng.seed(static_cast<unsigned>(pros::micros()));

  parts.assign(N_PARTICLES, {0, 0, 1.0 / N_PARTICLES});
  for (particle& p : parts) {
    p.x = x + gauss(seed_spread);
    p.y = y + gauss(seed_spread);
  }

  last_ox = chassis.odom_x_get();
  last_oy = chassis.odom_y_get();
  last_th = chassis.odom_theta_get();
  est_x = x;
  est_y = y;
  sprd_x = sprd_y = seed_spread;
  still_ticks = 0;
  accepted_count = gated_count = 0;
  for (bool& b : good_hits) b = false;
  good_idx = 0;
  flushes = 0;
  running = true;

  if (task == nullptr) task = new pros::Task(task_fn, "mcl");
  mtx.give();
}

void mcl::stop() {
  mtx.take();
  running = false;
  mtx.give();
}

void mcl::auto_flush_set(bool on) {
  mtx.take();
  auto_flush = on;
  mtx.give();
}

bool mcl::flush_if_safe() {
  mtx.take();
  bool applied = running ? flush_locked() : false;
  mtx.give();
  return applied;
}

// Telemetry -- single doubles; a torn read here costs a garbage screen frame,
// not a control action (same risk EZ's own screen task accepts).
double mcl::x() { return est_x; }
double mcl::y() { return est_y; }
double mcl::spread_x() { return sprd_x; }
double mcl::spread_y() { return sprd_y; }
double mcl::pending_x() { return est_x - chassis.odom_x_get(); }
double mcl::pending_y() { return est_y - chassis.odom_y_get(); }
bool mcl::confident_x() { return confident_axis(sprd_x); }
bool mcl::confident_y() { return confident_axis(sprd_y); }
int mcl::sensors_accepted() { return accepted_count; }
int mcl::sensors_gated() { return gated_count; }
int mcl::flush_count() { return flushes; }
