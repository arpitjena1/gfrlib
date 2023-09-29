#include "globals.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
//pistons
gfr::Solenoid wings1('A');
gfr::Solenoid wings2('B');
gfr::Solenoid intakep('C');


//motors
pros::Motor intake(9,true);
pros::Motor slapperm(10,true);



//sensors
pros::ADIDigitalIn limit('A');
pros::Optical opti(10);