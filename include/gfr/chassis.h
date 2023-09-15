#ifndef _GFR_CHASSIS_H_
#define _GFR_CHASSIS_H_

#include "gfr/flags.h"
#include "gfr/point.h"
#include "gfr/pose.h"
#include "gfr/asset.h"
#include "pose.h"
#include <memory>
#include "../api.h"
#include "Eigen/Eigen"

namespace gfr::chassis {

extern double maxSpeed;
extern std::shared_ptr<pros::Motor_Group> leftMotors;
extern std::shared_ptr<pros::Motor_Group> rightMotors;


/**
 * Set the brake mode for all chassis motors
 */
void setBrakeMode(pros::motor_brake_mode_e_t b);

/**
 * Return true of the chassis is not moving
 */
bool settled();

/**
 * Wait for the chassis to complete the current movement
 */
void waitUntilFinished(double exit_error);

/**
 * Perform 2D chassis movement
 */
void move(std::vector<double> target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);
void move(std::vector<double> target, double max, double exit_error,
          MoveFlags = NONE);
void move(std::vector<double> target, double max, MoveFlags = NONE);
void move(std::vector<double> target, MoveFlags = NONE);

/**
 * Perform 1D chassis movement
 */
void move(double target, double max, double exit_error,
          double lp, double ap, MoveFlags = NONE);
void move(double target, double max, double exit_error,
          MoveFlags = NONE);
void move(double target, double max, MoveFlags = NONE);
void move(double target, MoveFlags = NONE);

/**
 * Perform a turn movement
 */
void turn(double target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(double target, double max, double exit_error, MoveFlags = NONE);
void turn(double target, double max, MoveFlags = NONE);
void turn(double target, MoveFlags = NONE);

void chainedmoveto(std::vector<Pose> points, double max, MoveFlags flags );
/**
 * Turn to face a point
 */
void turn(Point target, double max, double exit_error, double ap,
          MoveFlags = NONE);
void turn(Point target, double max, double exit_error, MoveFlags = NONE);
void turn(Point target, double max, MoveFlags = NONE);
void turn(Point target, MoveFlags = NONE);

/**
 * follow a path on the sd card
 */
void follow(asset path, int timeout, float lookahead, bool reverse = false, float maxSpeed = 127,
                    bool log = false);
        /**
         * @brief Control the robot during the driver control period using the tank drive control scheme. In this
         * control scheme one joystick axis controls one half of the robot, and another joystick axis controls another.
         * @param left speed of the left side of the drivetrain. Takes an input from -127 to 127.
         * @param right speed of the right side of the drivetrain. Takes an input from -127 to 127.
         * @param curveGain control how steep the drive curve is. The larger the number, the steeper the curve. A value
         * of 0 disables the curve entirely.
         */

/**
 * Assign a power to the left and right motors
 */
void tank(double left, double right, bool velocity = false);

/**
 * Assign a vertical and horizontal power to the motors
 */
void arcade(double vertical, double horizontal, bool velocity = false);

/**
 * initialize the chassis
 */
void init(std::initializer_list<int8_t> leftMotors,
          std::initializer_list<int8_t> rightMotors, pros::motor_gearset_e_t gearset,
          double slew_step, double linear_exit_error, double angular_exit_error, 
          double settle_thresh_linear, double settle_thresh_angular,
          int settle_time);

} // namespace arms::chassis

#endif
