#include "main.h"

void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);


	while (true) {
		gfr::chassis::arcade(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));
		
	}
}