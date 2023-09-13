#include "globals.hpp"

pros::Controller master();
//pistons
gfr::Solenoid wings1('A');
gfr::Solenoid wings2('B');
gfr::Solenoid intakep('C');


//motors
pros::Motor intake(9,true);
pros::Motor slapper(10,true);



//sensors
pros::ADIDigitalIn limit('A');
pros::Optical opti;