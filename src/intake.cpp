#include "main.h"

// Negative port reverses the motor (replaces the old `true` reverse flag).
pros::Motor bottom_intake_left(-16, pros::MotorGears::blue);  // blue = 600 RPM (old _06)
pros::Motor top_intake_right(-17, pros::MotorGears::blue);

// Sets the bottom intake speed in autonomous.
void set_bottom_intake(int speed) {
  bottom_intake_left.move(speed);
}

// Driver control.  Call this every loop in opcontrol().
//   R1 = intake in   (both motors together, in unison)
//   R2 = reverse     (both motors together, the other way)
//   L1 = hold to drop the hood STOPPER down; release and it pops back up
void intake_control() {
  // ---- Intake + outtake run together in unison ----
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

  // ---- Hood = stopper.  Rests UP (extended); hold L1 to drop it DOWN
  //      (retracted); let go and it pops back UP. ----
  if (master.get_digital(DIGITAL_L1)) {
    hood.retract();  // down while L1 held
  } else {
    hood.extend();   // up otherwise (resting state)
  }
}
