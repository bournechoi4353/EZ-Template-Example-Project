# CLAUDE.md

VEX V5 robot project — **PROS 4.1.1** + **EZ-Template** (drivetrain, odometry, PID, pure pursuit). Team: **Pink Sparklee Unicorns** (BC2145).

## Build / flash

```bash
pros make          # compile
pros make clean    # REQUIRED after changing anything in static/*.txt (embedded paths)
pros mu            # make + upload
```

This environment's working PROS binary (the VS Code extension's bundle):
`"/Users/bournechoi/Library/Application Support/Code/User/globalStorage/sigbots.pros/install/pros-cli-macos/pros"`
(`prosv5` is **not** on PATH here — use that full path or the extension.)

Always build after edits; the project should stay green.

## Ports

Smart ports (1–21):

| Port | Device | Where |
|------|--------|-------|
| −3, 4, −5 | Left drive motors | main.cpp chassis ctor |
| 11, 12, −13 | Right drive motors | main.cpp chassis ctor |
| 6 | Horizontal tracking wheel (rotation, 3.25") | main.cpp `horiz_tracker` |
| −16 | Bottom intake (left) motor | intake.cpp |
| −17 | Top intake (right) motor | intake.cpp |
| 15 (−, reversed) | Vertical tracking wheel (rotation, 2.75") | main.cpp `vert_tracker` |
| 21 | IMU | main.cpp chassis ctor |
| 8 | Distance — back | subsystems.hpp |
| 18 | Distance — right | subsystems.hpp |
| 19 | Distance — left | subsystems.hpp |

ADI / 3-wire (pistons, `pros::adi::Pneumatics`, subsystems.hpp): **B** loader, **C** indexer, **D** wing, **E** hood, **F** antiDump, **G** lift, **H** descore. (Port A unused. No front distance sensor.)

PROS 4 Motor API note: reverse a motor with a **negative port**, gearset is `pros::MotorGears::blue` (600 RPM) — the old `E_MOTOR_GEARSET_06` + `bool` reverse arg won't compile.

## Odometry (2D: vertical + horizontal tracker + IMU)

Configured in main.cpp; **state lives inside the global `chassis` object** (no odom.cpp). Calibrated offsets:
- `vert_tracker(-15, 2.75, 1.05)` → `odom_tracker_left_set` (forward distance)
- `horiz_tracker(6, 3.25, -7.61)` → `odom_tracker_back_set` (lateral; mounted ~8" behind center)

Tuning constants are in `default_constants()` in autons.cpp (`pid_odom_angular_constants_set`, `odom_look_ahead_set`, etc.). Field half-width = **70.25"** (center → inner wall), used by wall resets.

### Hard-won odom gotchas (read before debugging odom)
- **Test odom drift with a MOTOR pivot ("Odom Spin"), never by hand.** Hand-rotating always slides the robot, so good odom *looks* broken (x/y "integrates up"). A clean motor pivot of ~3 rotations should leave x/y within ~1".
- **Horizontal-tracker offset sign:** Measure Offsets reported −7.61 and that's what keeps x/y put on a spin. If x/y runs away during a pure spin → flip the sign.
- **Verify tracker direction BEFORE Measure Offsets** ("Tracker Dir": push forward → vert up, push right → horiz up). A reversed tracker poisons the offset.
- **Seed heading near 90° makes odom drives go backward.** Seed `(0,0,0)` (facing forward) and place the robot facing forward — the proven-good convention.
- `pid_drive_set` / `pid_turn` use **drive-motor encoders + IMU** (not the trackers). The **trackers only drive `pid_odom_set` / pure pursuit**. Turns are equally accurate either way (same IMU).

## Modules

- **main.cpp** — chassis ctor (drive/IMU/tracker ports), odom enable, auton selector, opcontrol loop.
- **autons.cpp** — `default_constants()` (all PID/odom tuning) + every auton/test function. Speed consts: `DRIVE_SPEED 110`, `TURN_SPEED 90`.
- **intake.cpp / .hpp** — intake motors, `set_bottom_intake(int)`, `intake_control()` (driver).
- **subsystems.hpp** — pistons + distance sensors (`inline` globals; this is the "device" file).
- **wall_reset.cpp / .hpp** — distance-sensor wall resets (`wall_reset_all()`); only valid when squared to a known wall. Sensor offset ~6.3" (center→face).
- **motion_profile.cpp / .hpp** — `profiled_drive(in, max_vel, max_accel)`: standalone **trapezoidal** straight-line profile (borrows the motors, does NOT modify EZ-Template). kV/kA/kP need tuning per robot.
- **jerryio_path.cpp / .hpp + path_assets.S** — run path.jerryio paths via EZ pure pursuit (below).

## path.jerryio workflow

1. Design on path.jerryio, **export in CENTIMETERS** (the parser assumes cm → inches ÷2.54; the file doesn't record its unit).
2. Drop `static/<name>.txt`, add `PATH_ASSET <name>` to `src/path_assets.S` (use the 2-arg form if the filename has dots), declare `ASSET(<name>_txt)` in autons.cpp.
3. **`pros make clean`** (the build can't see an embedded `.txt` changed).
4. Follow it: `jerryio::follow_path(<name>_txt, DRIVE_SPEED, ...)`, then `chassis.pid_wait()`.

Key options:
- `start_forward=true` (+ seed `(0,0,0)`) rotates the whole path so it **starts going forward** — place the robot facing forward, no measuring start angle. Makes the path relative to start (so wall-reset/abs-field coords won't apply).
- `follow_path_reverse_tail(path, speed)` — for paths that **double back**: drives forward to the turnaround (heading flip >135°), then reverses the rest (e.g. back the output into a goal). ⚠️ if the path drives into a wall, the robot jams — keep the turnaround inside the field.

## Controls (opcontrol)

- **Tank** drive (left stick = left side, right stick = right side).
- **R1** intake in · **R2** intake reverse (both motors) · **L1** (hold) drop hood down, release = up.
- EZ extras (only off comp): **X** PID tuner, **DOWN+B** run selected auton.

## Autons (selector, in main.cpp)

Default = **"pushbacktest"** (first entry). Real autons: `pushbacktest`, `jerryio_path_example` (JerryIO Path), `BC2145AUTO` + `_fullsend` (drift), `pistons`.
Tuning/diagnostic tools: `Measure Offsets`, `Turn Test`, `Tracker Dir`, `Odom Spin`, `Odom Drive`, `Odom Sandbox`, `Motion Profile`, `Profiling Showcase`, `Wall Reset`.

## Conventions

- Comments explain the *why* (esp. the tuning/odom gotchas) — keep that style.
- Pistons: `.extend()` / `.retract()` / `.toggle()`. Hood: extend = up, retract = down. `pros::delay(ms)` to time things (3 s = `3000`).
- Park-in-a-`while(true)`-loop test autons keep their result on the brain screen; stop the program after reading.
