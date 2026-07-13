#include "main.h"

// ports in config.hpp. blue cartridge = 600rpm
pros::Motor bottom_intake_left(BOTTOM_INTAKE_PORT, pros::MotorGears::blue);
pros::Motor top_intake_right(TOP_INTAKE_PORT, pros::MotorGears::blue);

void set_bottom_intake(int speed) {
  bottom_intake_left.move(speed);
}

// R1 = in, R2 = out, hold L1 to drop the hood. call this every loop in opcontrol
void intake_control() {
  if (master.get_digital(DIGITAL_R1)) {
    bottom_intake_left.move(127);
    top_intake_right.move(127);
  } else if (master.get_digital(DIGITAL_R2)) {
    bottom_intake_left.move(-127);
    top_intake_right.move(-127);
  } else {
    bottom_intake_left.move(0);
    top_intake_right.move(0);
  }

  // hood rests up, drops while L1 is held
  if (master.get_digital(DIGITAL_L1)) {
    hood.retract();
  } else {
    hood.extend();
  }
}
