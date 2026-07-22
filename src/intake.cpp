#include "main.h"

// port in config.hpp. blue cartridge = 600rpm (change if your intake is geared differently)
pros::Motor intake(INTAKE_PORT, pros::MotorGears::blue);

void set_intake(int speed) {
  intake.move(speed);
}

// hold R1 = in, hold R2 = out, release = stop. call this every loop in opcontrol
void intake_control() {
  if (master.get_digital(DIGITAL_L1)) {
    intake.move(127);   // in
  } else if (master.get_digital(DIGITAL_L2)) {
    intake.move(-127);  // out
  } else {
    intake.move(0);     // stop
  }
}
