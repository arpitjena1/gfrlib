#include "gfr/api.h"
#include "api.h"
#include "pros/motors.h"

#include <tuple>

namespace gfr::chassis {

void moveChassis(float linearVelocity, float angularVelocity) {
    // compute left and right velocities
    float leftVelocity = (2 * linearVelocity - angularVelocity * TRACK_WIDTH) / 2; // inches/sec
    float rightVelocity = (2 * linearVelocity + angularVelocity * TRACK_WIDTH) / 2; // inches/sec

    // calculate left and right wheel rpm
    float leftRPM = leftVelocity * 60.0 / (WHEEL_DIAM * M_PI); // rpm
    float rightRPM = rightVelocity * 60.0 / (WHEEL_DIAM * M_PI); // rpm

    // calculate the left and right motor rpm(gear ratio)
    float leftMotorRPM = leftRPM * (3/5);
    float rightMotorRPM = rightRPM * (3/5);

    // move chassis
    leftMotors->move_velocity(leftMotorRPM);
    rightMotors->move_velocity(rightMotorRPM);
}

double distancecalc(Pose a , Pose b ){
	return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

double anglecalc(Pose a, Pose b){
	double targetAngle = atan2(b.y - a.y, b.x - a.x);
	return gfr::util::radToDeg(targetAngle);
	
}
double wrap180(double deg)
{
    while (deg > 180 || deg < -180)
    {
        if (deg > 180)
        {
            deg -= 360;
        }

        else if (deg < -180)
        {
            deg += 360;
        }
    }

    return deg;
}


// chassis motors
std::shared_ptr<pros::Motor_Group> leftMotors;
std::shared_ptr<pros::Motor_Group> rightMotors;

// slew control (autonomous only)
double slew_step; // smaller number = more slew

// default exit error
double linear_exit_error;
double angular_exit_error;

// settling
double settle_thresh_linear;
double settle_thresh_angular;
int settle_time;

// chassis variables
double maxSpeed = 100;
double leftPrev = 0;
double rightPrev = 0;
double leftDriveSpeed = 0;
double rightDriveSpeed = 0;

/**************************************************/
// motor control
void motorMove(std::shared_ptr<pros::Motor_Group> motor, double speed,
               bool velocity) {
	if (velocity)
		motor->move_velocity(speed * (double)motor->get_gearing()[0] / 100);
	else
		motor->move_voltage(speed * 120);

	if (motor == leftMotors)
		leftPrev = speed;
	else
		rightPrev = speed;
}

void setBrakeMode(pros::motor_brake_mode_e_t b) {
	leftMotors->set_brake_modes((pros::motor_brake_mode_e_t)b);
	rightMotors->set_brake_modes((pros::motor_brake_mode_e_t)b);
	motorMove(leftMotors, 0, true);
	motorMove(rightMotors, 0, true);
}

/**************************************************/
// speed control
double limitSpeed(double speed, double max) {
	if (speed > max)
		speed = max;
	if (speed < -max)
		speed = -max;

	return speed;
}

double slew(double target_speed, double step, double current_speed) {

	if (fabs(current_speed) > fabs(target_speed))
		step = 200;

	if (target_speed > current_speed + step)
		current_speed += step;
	else if (target_speed < current_speed - step)
		current_speed -= step;
	else
		current_speed = target_speed;

	return current_speed;
}


/**************************************************/
// settling
bool settled() {
	// previous position values
	static Point p_pos = {0, 0};
	static double p_ang = 0;

	static int settle_count = 0;

	Point pos = odom::getPosition();
	double ang = odom::getHeading();

	if (fabs(pos.x - p_pos.x) > settle_thresh_linear) {
		p_pos.x = pos.x;
		settle_count = 0;
	} else if (fabs(pos.y - p_pos.y) > settle_thresh_linear) {
		p_pos.y = pos.y;
		settle_count = 0;
	} else if (fabs(ang - p_ang) > settle_thresh_angular) {
		p_ang = ang;
		settle_count = 0;
	} else {
		settle_count += 10;
	}

	if (settle_count > settle_time)
		return true;
	else
		return false;
}

void waitUntilFinished(double exit_error) {
	pros::delay(400); // minimum movement time
	switch (pid::mode) {
	case TRANSLATIONAL:
		while (odom::getDistanceError(pid::pointTarget) > exit_error &&
		       !settled()) {
			pros::delay(10);
		}

		// if doing a pose movement, make sure we are at the target theta
		if (pid::angularTarget != 361) {
			while (fabs(odom::getHeading() - pid::angularTarget) > exit_error &&
			       !settled())
				pros::delay(10);
		}

		break;
	case ANGULAR:
		while (fabs(odom::getHeading() - pid::angularTarget) > exit_error &&
		       !settled())
			pros::delay(10);
		break;
	}
}

/**************************************************/
// 2D movement
void move(std::vector<double> target, double max, double exit_error, double lp,
          double ap, MoveFlags flags) {
	pid::mode = TRANSLATIONAL;

	double x = target.at(0);
	double y = target.at(1);
	double theta =
	    target.size() == 3 ? fmod(target.at(2), 360) : 361; // setinel value

	if (flags & RELATIVE) {
		Point p = odom::getPosition();     // robot position
		double h = odom::getHeading(true); // robot heading in radians
		double x_new = p.x + x * cos(h) - y * sin(h);
		double y_new = p.y + x * sin(h) + y * cos(h);
		x = x_new;
		y = y_new;
		if (target.size() == 3)
			theta += fmod(odom::getHeading(), 360);
	}

	pid::pointTarget = Point{x, y};
	pid::angularTarget = theta;

	maxSpeed = max;
	pid::linearKP = lp;
	pid::trackingKP = ap;
	pid::thru = (flags & THRU);
	pid::reverse = (flags & REVERSE);
	pid::canReverse = false;

	// reset the integrals
	pid::in_lin = 0;
	pid::in_ang = 0;

	if (!(flags & ASYNC)) {
		waitUntilFinished(exit_error);
		pid::mode = DISABLE;
		if (!(flags & THRU))
			chassis::setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	}
}
float getCurvature(gfr::Pose pose, gfr::Pose other) {

    // calculate whether the pose is on the left or right side of the circle
    float side = gfr::util::sgn(std::sin(pose.theta) * (other.x - pose.x) - std::cos(pose.theta) * (other.y - pose.y));
    // calculate center point and radius
    float a = -std::tan(pose.theta);
    float c = std::tan(pose.theta) * pose.x - pose.y;
    float x = std::fabs(a * other.x + other.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.x - pose.x, other.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}
void move(float x, float y, float theta, bool forwards, int timeout, float chasePower, float lead, float maxSpeed){
	Pose target(x, y, M_PI - gfr::util::degToRad(theta));
	gfr::PID linearPID = PID(0, 0, LINEAR_KP, 0, LINEAR_KD, "linearPID"); // linear PID controller
    gfr::PID angularPID = PID(0, 0, ANGULAR_KP, 0, ANGULAR_KD, "angularPID"); // angular PID controller

	if (!forwards) target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement
	bool close = false; // used for settling
	// main loop
    while (true) {
        // get current pose
        Pose pose = gfr::odom::getPose(true);
        if (!forwards) pose.theta += M_PI;
        pose.theta = M_PI - pose.theta; // convert to standard form

        // check if the robot is close enough to the target to start settling
        if (pose.distance(target) < 7.5) close = true;

        // calculate the carrot point
        Pose carrot = target - (Pose(cos(target.theta), sin(target.theta)) * lead * pose.distance(target));
        if (close) carrot = target; // settling behavior

        // calculate error
        float angularError = gfr::util::angleError(pose.angle(carrot), pose.theta, true); // angular error
        float linearError = pose.distance(carrot) * cos(angularError); // linear error
        if (close) angularError = gfr::util::angleError(target.theta, pose.theta, true); // settling behavior
        if (!forwards) linearError = -linearError;

        // get PID outputs
        float angularPower = -angularPID.update(gfr::util::radToDeg(angularError), 0);
        float linearPower = linearPID.update(linearError, 0);

        // calculate radius of turn
        float curvature = fabs(getCurvature(pose, carrot));
        if (curvature == 0) curvature = -1;
        float radius = 1 / curvature;

        // calculate the maximum speed at which the robot can turn
        // using the formula v = sqrt( u * r * g )
        if (radius != -1) {
            float maxTurnSpeed = sqrt(chasePower * radius * 9.8);
            // the new linear power is the minimum of the linear power and the max turn speed
            if (linearPower > maxTurnSpeed && !close) linearPower = maxTurnSpeed;
            else if (linearPower < -maxTurnSpeed && !close) linearPower = -maxTurnSpeed;
        }

        // prioritize turning over moving
        float overturn = fabs(angularPower) + fabs(linearPower) - maxSpeed;
        if (overturn > 0) linearPower -= linearPower > 0 ? overturn : -overturn;

        // calculate motor powers
        float leftPower = linearPower + angularPower;
        float rightPower = linearPower - angularPower;

        // move the motors
        gfr::chassis::leftMotors->move(leftPower);
        gfr::chassis::rightMotors->move(rightPower);

        pros::delay(10); // delay to save resources
	}
	gfr::chassis::leftMotors->move(0);
    gfr::chassis::rightMotors->move(0);
}

void move(std::vector<double> target, double max, double exit_error,
          MoveFlags flags) {
	move(target, max, exit_error, -1, -1, flags);
}

void move(std::vector<double> target, double max, MoveFlags flags) {
	move(target, max, linear_exit_error, -1, -1, flags);
}

void move(std::vector<double> target, MoveFlags flags) {
	move(target, 100, linear_exit_error, -1, -1, flags);
}

/**************************************************/
// 1D movement
void move(double target, double max, double exit_error, MoveFlags flags) {
	move({target, 0}, max, exit_error, -1, -1, flags | RELATIVE);
}

void move(double target, double max, MoveFlags flags) {
	move({target, 0}, max, linear_exit_error, -1, -1, flags | RELATIVE);
}

void move(double target, MoveFlags flags) {
	move({target, 0}, 100, linear_exit_error, -1, -1, flags | RELATIVE);
}

/**************************************************/
// rotational movement
void turn(double target, double max, double exit_error, double ap,
          MoveFlags flags) {
	pid::mode = ANGULAR;

	double bounded_heading = (int)(odom::getHeading()) % 360;

	double diff = target - bounded_heading;

	if (diff > 180)
		diff -= 360;
	else if (diff < -180)
		diff += 360;

	if (flags & RELATIVE) {
		diff = target;
	}

	double true_target = diff + odom::getHeading();

	pid::angularTarget = true_target;
	maxSpeed = max;
	pid::angularKP = ap;
	pid::in_ang = 0; // reset the integral value to zero

	if (!(flags & ASYNC)) {
		waitUntilFinished(exit_error);
		pid::mode = DISABLE;
		if (!(flags & THRU))
			chassis::setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	}
}

void turn(double target, double max, double exit_error, MoveFlags flags) {
	turn(target, max, exit_error, -1, flags);
}

void turn(double target, double max, MoveFlags flags) {
	turn(target, max, angular_exit_error, -1, flags);
}

void turn(double target, MoveFlags flags) {
	turn(target, 100, angular_exit_error, -1, flags);
}

/**************************************************/
// ramsete


gfr::Pose keepvelcheck(0,0,0);
void ramsete(gfr::Pose targetPose, gfr::Pose currentPose, float targetAngularVelocity, float targetLinearVelocity, float beta, float zeta) {
    // compute global error
    Eigen::MatrixXd globalError(1, 3);
    globalError <<
        targetPose.x - currentPose.x,
        targetPose.y - currentPose.y,
        targetPose.theta - currentPose.theta;

    // compute transformation matrix
    Eigen::MatrixXd transformationMatrix(3, 3);
    transformationMatrix <<
        cos(currentPose.theta),  sin(currentPose.theta), 0,
        -sin(currentPose.theta), cos(currentPose.theta), 0,
        0,                       0,                      1;

    // compute local error
    Eigen::MatrixXd localError = globalError * transformationMatrix;
    // compute k gain
    float k = 2 * zeta * std::sqrt(targetAngularVelocity * targetAngularVelocity + beta + targetLinearVelocity * targetLinearVelocity);
    // compute angular velocity
    float angularVelocity = targetAngularVelocity * cos(localError(0, 2)) + k * localError(0, 0);
    // compute linear velocity
    float linearVelocity = targetLinearVelocity + k * localError(0, 2) + (beta * linearVelocity * sin(localError(0, 2)) * localError(0, 1) / localError(0, 2));
	
	keepvelcheck.returnvalues(angularVelocity, linearVelocity);
    // move chassis
    moveChassis(linearVelocity, angularVelocity);
}
int findClosest(gfr::Pose pose, std::vector<gfr::Pose>* pPath, int prevCloseIndex=0) {
    //Find the closest point to the robot
    int closeIndex = 0;
    float minDistance = INT_MAX;
    for(int i = prevCloseIndex; i<pPath->size(); i++){
        float dist = pose.distance(pPath->at(i));
        if(dist < minDistance){
            closeIndex = i;
            minDistance = dist;
        }
    }
    return closeIndex;
}
void FollowPath(std::vector<gfr::Pose>* pPath, float timeOut, float errorRange, float beta, float zeta, bool reversed = false){
    float offFromPose = INT_MAX;
    
    // set up the timer
    timeOut *= CLOCKS_PER_SEC;
    clock_t startTime = clock(); 
    float runtime = 0;

    // initialise loop variables
    int prevCloseIndex=0;
	

    // keep running the controller until either time expires or the bot is within the error range
    while(runtime <= timeOut && offFromPose >= errorRange){
        // update runtime
        runtime = clock() - startTime;

		//current pose
        Pose pose = gfr::odom::getPose(false);


        // find the closest index
        int closeIndex = findClosest(pose, pPath, prevCloseIndex);

        auto closestPose = pPath->at(closeIndex);

        
        // set the desired pose to one ahead (so the robot is always moving forward) *****TEST******
        int targetIndex = std::min(closeIndex+1, (int)pPath->size()-1); // ensures no out of range error
        gfr::Pose targetPose = pPath->at(targetIndex);

		float targetAngularVelocity = keepvelcheck.x;
		float targetLinearVelocity = keepvelcheck.y;
        // run the controller function
        ramsete(targetPose, pose, targetAngularVelocity, targetLinearVelocity, beta, zeta);

        pros::delay(20);
    }
}

/**************************************************/
// turn to point
void turn(Point target, double max, double exit_error, double ap,
          MoveFlags flags) {
	double angle_error = odom::getAngleError(target);
	turn(angle_error, max, exit_error, ap, flags);
}

void turn(Point target, double max, double exit_error, MoveFlags flags) {
	turn(target, max, exit_error, -1, flags);
}

void turn(Point target, double max, MoveFlags flags) {
	turn(target, max, angular_exit_error, -1, flags);
}

void turn(Point target, MoveFlags flags) {
	turn(target, 100, angular_exit_error, -1, flags);
}



/**************************************************/
// task control
int chassisTask() {
	while (1) {
		pros::delay(10);

		std::array<double, 2> speeds = {0, 0}; // left, right

		if (pid::mode == TRANSLATIONAL)
			speeds = pid::translational();
		else if (pid::mode == ANGULAR)
			speeds = pid::angular();
		else
			speeds = {leftDriveSpeed, rightDriveSpeed};

		// speed limiting
		speeds[0] = limitSpeed(speeds[0], maxSpeed);
		speeds[1] = limitSpeed(speeds[1], maxSpeed);

		// slew
		speeds[0] = slew(speeds[0], slew_step, leftPrev);
		speeds[1] = slew(speeds[1], slew_step, rightPrev);

		// output
		motorMove(leftMotors, speeds[0], false);
		motorMove(rightMotors, speeds[1], false);
	}
}

/**************************************************/
// initialization
void init(std::initializer_list<int8_t> leftMotors,
          std::initializer_list<int8_t> rightMotors,
          pros::motor_gearset_e_t gearset, double slew_step,
          double linear_exit_error, double angular_exit_error,
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time) {

	// assign constants
	chassis::slew_step = slew_step;
	chassis::linear_exit_error = linear_exit_error;
	chassis::angular_exit_error = angular_exit_error;
	chassis::settle_thresh_linear = settle_thresh_linear;
	chassis::settle_thresh_angular = settle_thresh_angular;
	chassis::settle_time = settle_time;

	// configure chassis motors
	chassis::leftMotors =
	    std::make_shared<pros::Motor_Group>(std::vector<int8_t>(leftMotors));
	chassis::rightMotors =
	    std::make_shared<pros::Motor_Group>(std::vector<int8_t>(rightMotors));
	chassis::leftMotors->set_gearing(gearset);
	chassis::rightMotors->set_gearing(gearset);

	pros::Task chassis_task(chassisTask);
}

/**************************************************/
// operator control
void tank(double left_speed, double right_speed, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous tasks
	maxSpeed = 100;
	chassis::leftDriveSpeed = left_speed;
	chassis::rightDriveSpeed = right_speed;
}

void arcade(double vertical, double horizontal, bool velocity) {
	pid::mode = DISABLE; // turns off autonomous task
	maxSpeed = 100;
	chassis::leftDriveSpeed = vertical + horizontal;
	chassis::rightDriveSpeed = vertical - horizontal;
}


} // namespace gfr::chassis
