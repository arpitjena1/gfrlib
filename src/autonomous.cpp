#include "main.h"


ASSET (path_txt);


void testing() {
	using namespace gfr;
	using namespace gfr::chassis;

	//pure pursuit
	follow(path_txt,3000,10,true);


	//boomerang
	move({{24,24,90}}, 127, gfr::ASYNC|gfr::THRU);
	intake.move(127);
	waitUntilFinished(2);


	//ramsete
	std::vector<gfr::Pose> *ramseteposes;
	
	Pose pose1 = {1,2,3};
	ramseteposes->push_back(pose1);

	FollowPath(ramseteposes,3000,200,10,10,false);

	//motion profile
	std::vector<gfr::Pose> *motionprofileposes;
	Pose posefirst = {1,2,3};
	motionprofileposes->push_back(posefirst);

	MotionProfile p(motionprofileposes, 30,30,30,10);

}

void leftsafe(){
	using namespace gfr;
	using namespace gfr::chassis;

}
void rightsafe(){
	using namespace gfr;
	using namespace gfr::chassis;
}

void leftsteal(){
	using namespace gfr;
	using namespace gfr::chassis;
}
void rightsteal(){
	using namespace gfr;
	using namespace gfr::chassis;

	
}
void autonomous(){
	clock_t startTime = clock(); 
	if(selector::auton == 1){

		leftsafe(); 
		
	}
	if(selector::auton == 2){
		rightsafe();
	}
	if(selector::auton == 3){
		leftsteal();
	}
	if(selector::auton == 4){
		rightsteal();
	}
	clock_t endTime = clock();
	master.print(3, 2, "auton time: %clock_t", (endTime- startTime));
}