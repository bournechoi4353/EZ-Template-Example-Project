#include "mcl_checkpoint.hpp"
#include "mcl.hpp"
#include "main.h"

void mcl_checkpoint(double target_x, double target_y, int max_wait_ms) {
    int waited = 0;
    const int poll_ms = 20;

    while (waited < max_wait_ms) {
        if (mcl::confident_x() || mcl::confident_y()) break;
        pros::delay(poll_ms);
        waited += poll_ms;
    }

    mcl::flush_if_safe();

    double actual_x = chassis.odom_x_get();
    double actual_y = chassis.odom_y_get();
    double error_x = target_x - actual_x;
    double error_y = target_y - actual_y;

    printf("[checkpoint] target=(%.1f, %.1f) actual=(%.1f, %.1f) error=(%.2f, %.2f)\n",
           target_x, target_y, actual_x, actual_y, error_x, error_y);
}