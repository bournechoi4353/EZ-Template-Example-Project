#pragma once

// wait (up to max_wait_ms) for MCL to get confident, flush the correction,
// then print how far odom is from where the auton says we should be
void mcl_checkpoint(double target_x, double target_y, int max_wait_ms = 400);
