#include "autonomous.hpp"


ASSET (path_txt);
void autonomous() {
	using namespace gfr;
	using namespace gfr::chassis;

	//pure pursuit
	follow(path_txt,3000,10,true);


	//boomerang
	move({{24,24,90}}, 127, gfr::ASYNC|gfr::THRU);
	intake.move(127);
	waitUntilFinished(2);

	//ramsete
	chainedmoveto({{Pose{10,10,10}},Pose(20,20,20)},100,gfr::THRU);

	
}