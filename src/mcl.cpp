#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "field_map.hpp"
#include "main.h"

// MCL engine -- see mcl.hpp for the big picture (IMU owns heading, MCL owns
// position). a few things in here look weird on purpose, don't "clean" them:
//  - particles are (x, y) only, heading comes off the IMU every tick. ~1 deg
//    of heading uncertainty is folded into the beam sigma instead
//  - the beam model has a flat outlier floor so one blocked beam (cup/robot)
//    can't zero out a good particle
//  - the innovation gate widens with cloud spread. a fixed gate would reject
//    the exact wall readings it needs to recover after a collision
//  - short readings only get half authority -- that's cup territory
//  - no process noise at rest. diversity comes from the resample jitter,
//    otherwise repeated identical readings collapse the cloud onto a few
//    duplicated particles
//  - x and y flush independently, near one wall only one axis is observable

namespace {

// tunables -- retune on the real robot
constexpr int N_PARTICLES = 300;   // drop to 150-200 if sharing CPU with AI vision
constexpr int LOOP_MS = 33;        // ~30 Hz

// motion model noise
constexpr double K_TRANS = 0.015;         // translational noise per inch traveled (~1.5%)
constexpr double K_TURN_XY = 0.010;       // extra x/y noise per degree turned (tracker wobble)
constexpr double RESAMPLE_JITTER = 0.15;  // jitter (in) injected at resample

// beam model
constexpr double SIGMA_BASE = 1.5;       // base range sigma (in)
constexpr double SIGMA_HEADING = 0.0175; // ~1 deg of shared-heading error, as rad*range
constexpr double OUTLIER_FLOOR = 0.08;   // flat floor relative to the gaussian peak
constexpr double GATE_SIGMAS = 3.0;      // innovation gate width
constexpr double GATE_PAD = 1.0;         // + fixed pad (in)

// sensor validity, same gates wall_reset uses
constexpr int CONF_MIN = 30;         // pros::Distance confidence is 0-63
constexpr double RANGE_MIN = 2.0;    // inches
constexpr double RANGE_MAX = 36.0;

// authority ramps 0.5 -> 1.0 between these readings (in)
constexpr double AUTH_LOW = 8.0;
constexpr double AUTH_FULL = 24.0;

// correction gates
constexpr double CONF_SPREAD = 3.0;   // per-axis cloud 1-sigma must be under this (in)
constexpr int GOOD_HITS_MIN = 5;      // and at least this many good updates in the window
constexpr int GOOD_HITS_WINDOW = 15;
constexpr double CORR_MIN = 0.25;     // ignore corrections smaller than this (in)
constexpr double CORR_MAX = 8.0;      // clamp per flush -- bigger pending = alarm
constexpr int STILL_TICKS = 5;        // settled = this many still ticks in a row (~165ms)
constexpr double STILL_POS = 0.05;    // per-tick "still" thresholds
constexpr double STILL_THETA = 0.2;   // degrees

constexpr double DEG2RAD = M_PI / 180.0;

// state
struct particle {
  double x, y, w;
};

std::vector<particle> parts;
std::vector<mcl::sensor_cfg> sensors;  // defaults filled in on first start()

pros::Mutex mtx;          // guards everything below (tick vs flush race)
pros::Task* task = nullptr;
bool running = false;
bool auto_flush = true;

double last_ox = 0, last_oy = 0, last_th = 0;  // odom at the previous tick
double est_x = 0, est_y = 0;                   // weighted-mean estimate
double sprd_x = 99, sprd_y = 99;               // per-axis cloud 1-sigma
int still_ticks = 0;
int accepted_count = 0, gated_count = 0;
bool good_hits[GOOD_HITS_WINDOW] = {};         // ring buffer: "had accepted beams"
int good_idx = 0;
int flushes = 0;

std::mt19937 rng(12345);  // fixed seed = reproducible bench runs. reseeded from micros() in start()

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

// one accepted beam, precomputed for the per-particle loop. heading is
// shared, so the rotated offset and ray direction are the same for every
// particle this tick
struct beam {
  double reading;      // inches
  double fox, foy;     // sensor face offset rotated into world frame
  double dirx, diry;   // world-frame unit ray
  double sigma;
  double auth;         // 0.5..1.0 weight authority
};

// weighted mean + per-axis spread from the cloud
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

// low-variance resample + jitter (the jitter keeps diversity, see note up top)
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
    p.x += gauss(RESAMPLE_JITTER);
    p.y += gauss(RESAMPLE_JITTER);
    p.w = step;
    next.push_back(p);
  }
  parts.swap(next);
}

// apply the pending correction if the gates pass. caller holds mtx.
// this is the ONE place odom ever gets written, and it's only ever x/y
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
  // the set jumps odom, so shift the motion model's reference too -- otherwise
  // next tick's delta would move the particles by the correction again
  last_ox += dx;
  last_oy += dy;
  flushes++;
  printf("[mcl] correction applied: (%+.2f, %+.2f)\n", dx, dy);
  return true;
}

void tick() {
  mtx.take();

  // odom in, deltas out
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

  // motion model: shift the cloud by the odom delta, plus noise if moving
  if (!still) {
    double s = K_TRANS * dist + K_TURN_XY * std::fabs(dth);
    for (particle& p : parts) {
      p.x += dx + gauss(s);
      p.y += dy + gauss(s);
    }
  } else {
    for (particle& p : parts) {
      p.x += dx;  // sub-threshold creep still tracks, no noise at rest
      p.y += dy;
    }
  }

  // read sensors and run the gates
  double h = th * DEG2RAD;
  double c = std::cos(h), s = std::sin(h);
  std::vector<beam> beams;
  accepted_count = 0;
  gated_count = 0;

  for (const mcl::sensor_cfg& sc : sensors) {
    double reading = sc.dev->get() / 25.4;  // mm -> in
    if (sc.dev->get_confidence() < CONF_MIN) continue;
    if (reading < RANGE_MIN || reading > RANGE_MAX) continue;

    // body -> world (the one trig rule: heading h -> (sin h, cos h))
    beam b;
    b.reading = reading;
    b.fox = sc.off_x * c + sc.off_y * s;
    b.foy = -sc.off_x * s + sc.off_y * c;
    double a = h + sc.facing_deg * DEG2RAD;
    b.dirx = std::sin(a);
    b.diry = std::cos(a);

    // innovation gate vs the current estimate, widened by cloud spread along
    // the ray -- stays open after a collision so recovery is possible, tight
    // when confident (a cup 12" in front of the wall gets dropped here)
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

    // short readings are cup territory -- half authority
    b.auth = 0.5 + 0.5 * std::clamp((reading - AUTH_LOW) / (AUTH_FULL - AUTH_LOW), 0.0, 1.0);
    beams.push_back(b);
  }
  accepted_count = static_cast<int>(beams.size());
  good_hits[good_idx] = !beams.empty();
  good_idx = (good_idx + 1) % GOOD_HITS_WINDOW;

  // weight, estimate, resample (only if we actually measured something)
  if (!beams.empty()) {
    double wsum = 0;
    for (particle& p : parts) {
      double lw = 1.0;
      for (const beam& b : beams) {
        double pred = field::raycast(p.x + b.fox, p.y + b.foy, b.dirx, b.diry);
        double f;
        if (pred < 0) {
          f = OUTLIER_FLOOR;  // drifted outside the field, let it die
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
      // every beam disagreed with every particle -- reset the weights instead
      // of dividing by zero. the innovation gate makes this rare
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
    // nothing measured -- coast, the cloud already shifted with odom
    refresh_estimate();
  }

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

// public API

void mcl::sensors_set(const std::vector<mcl::sensor_cfg>& s) {
  mtx.take();
  sensors = s;
  mtx.give();
}

void mcl::start(double x, double y, double seed_spread) {
  mtx.take();

  // measured sensor geometry (body frame, facing CW from forward). off_y
  // only matters for the back sensor -- it aims -Y so its setback biases the
  // range; the side sensors only care about off_x. beam_height is just a
  // note, but 9-10" is high and may shoot over a short field wall, which
  // shows up as gated side beams
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

// telemetry, plain double reads -- a torn read costs one garbage screen
// frame, not a control action
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
