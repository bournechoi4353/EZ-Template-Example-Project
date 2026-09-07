#pragma once

// all the PID/odom tuning, gets called in initialize()
void default_constants();

// live capture tool: jog the arm by hand, screen shows the current encoder
// position so you can write down the number at each pin height (0 = rest)
void arm_height_capture();

// ---- match autons ----
void redLeft();
void redRight();
void blueLeft();
void blueRight();

// programming skills run
void skills();
