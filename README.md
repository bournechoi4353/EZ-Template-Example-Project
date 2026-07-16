# VEX V5 Robot Code

Competition code for our VEX V5 robot. Built on PROS 4 and EZ-Template, which handle the drivetrain, odometry, PID, and pure pursuit.

## Building

Compile with `pros make`, or `pros mu` to compile and upload to the brain. If you change any of the path files in `static/`, run `pros make clean` first — the build embeds those files and won't notice they changed otherwise.

## Layout

Chassis setup, the auton selector, and driver control are in `src/main.cpp`. All the autons and PID tuning constants live in `src/autons.cpp`. Pistons and sensors are declared in `include/subsystems.hpp`, and the intake has its own file. The rest is supporting stuff: wall resets off the distance sensors, a standalone motion profiler, a runner for path.jerryio exports, and a Monte Carlo localization module that acts as an automatic wall reset during autons.

## Controls

Tank drive. R1 runs the intake, R2 reverses it, and holding L1 drops the hood. When not connected to competition control, X opens the PID tuner and DOWN+B runs the selected auton.

## Autons

The selector shows up on the brain screen. The first entry is the main match routine; most of the rest are tuning and diagnostic tools. The test autons sit in a loop so their results stay on screen — just stop the program when you're done reading.

thanks claudecode in the development of the MCL, creation of the "drifting" pattern and finding ratios for more advanced movements for me to use, teaching me how to use pneumatics, commenting, debugging, and teaching me how to develop sensor calibrations. 
