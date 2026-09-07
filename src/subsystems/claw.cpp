#include "main.h"

// roller: 2x 5.5W motors as ONE group. green gearset (5.5W motors report as
// green). sign is a default, flip the ports in ports.hpp if it spins wrong.
pros::MotorGroup roller({ROLLER_RIGHT_PORT, ROLLER_LEFT_PORT}, pros::MotorGears::green);
pros::Motor rotationClaw(1, pros::MotorGears::green);
// claw clamp piston: open/close. starts retracted (closed).
pros::adi::Pneumatics clawPiston(CLAW_PISTON_PORT, false);

// positive = in
void set_roller(int speed) {
  roller.move(speed);
}
bool rotatedClaw = false;
void rotateClaw() {
rotatedClaw = !rotatedClaw;
if (rotatedClaw == true) {
  rotationClaw.move_absolute(-1350, 110);
} else if (rotatedClaw == false) {
  rotationClaw.move_absolute(0, 90);
}
}
