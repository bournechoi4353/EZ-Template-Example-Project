#include "main.h"

// single motor. green cartridge = 200rpm (change if geared differently)
pros::MotorGroup arm({LIFT_PORT, LIFT2_PORT}, pros::MotorGear::green);
pros::Rotation arm_sensor(ARM_SENSOR_PORT);

// positive = up
void set_arm(int speed) {
  double pos = arm_sensor.get_position() / 100.0;
  if (pos >= ARM_SENSOR_UPPER_LIMIT && speed > 0) speed = 0;
  if (pos <= ARM_SENSOR_LOWER_LIMIT && speed < 0) speed = 0;
  arm.move(speed);
}

// hard stop for auton moves -- arm.move_absolute() bypasses set_arm(), so this
// task watches arm_sensor directly and kills arm motion the instant it's
// outside [ARM_SENSOR_LOWER_LIMIT, ARM_SENSOR_UPPER_LIMIT], no matter what
// commanded it.
void arm_limit_task_fn() {
  while (true) {
    double pos = arm_sensor.get_position() / 100.0;
    double vel = arm.get_actual_velocity();
    if ((pos >= ARM_SENSOR_UPPER_LIMIT && vel > 0) ||
        (pos <= ARM_SENSOR_LOWER_LIMIT && vel < 0)) {
      arm.move(0);
    }

    chassis.opcontrol_speed_max_set(pos > ARM_SPEED_LIMIT_THRESHOLD ? ARM_SPEED_LIMIT : 127);

    pros::delay(10);
  }
}
pros::Task arm_limit_task(arm_limit_task_fn);