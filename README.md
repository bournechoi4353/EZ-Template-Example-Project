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


## AI usage

so basically, I used this project as a "learning playground" where i learned from the basics of C++ to utilizing ez-template and making the robot move, and then finally learning how MCL and other heavy concepts work. If you look at some of the code, you can see that I hand-coded it, particularly paths and stuff since I will have to be proficient in it when it comes to competition robotics. however, for more complex things such as MCL, wall resets, ai sensors, etc... I used AI to code the things out, and I then analyzed that code so I can learn how to use all of it. As someone who has a background of coding, I understand the very basics of every language since their inherently similar, it's just that since I've never touched C++ I didn't know how to do it.

Here's how I used the claude-coded stuff to learn: So after basically generating the complex code (simple one's I fully understood) I pasted it into a seperate chat, and asked it to translate it into Python (a language I'm comfortable with), so that I can fully understand the mechanics. Through this, I learned that in the end, all code is structured in a similar way, its just that C++ is much more complex and more "connected" to the computer compared to python and stuff which is a layer on top. I also learned how MCL works pretty well, how offsets for odometry sensors/trackers work, how to plan out paths efficiently, how to set my PID constants quickly, and so much more. So that's why I utilized so much AI in this project, but I got some notes on PID to be exact (which I tested and learned from here) and some hand coded stuff for the paths, and learned how MCL works (basically a version of SLAM that utilizes prediction from distance sensors).this was more of a learning experience/learning platform for me rather than a product like a game.
