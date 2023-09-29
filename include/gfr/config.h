#ifndef _ARMS_CONFIG_H_
#define _ARMS_CONFIG_H_

#include "gfr/api.h"


namespace gfr {

// Debug
#define ODOM_DEBUG 0

// Negative numbers mean reversed motor
#define LEFT_MOTORS 1, 2
#define RIGHT_MOTORS -3, -4
#define GEARSET pros::E_MOTOR_GEAR_600 // RPM of chassis motors
#define WHEEL_DIAM 3.25

// Ticks per inch
#define TPI 88.1473531      			  // Encoder ticks per inch of forward robot movement
#define MIDDLE_TPI 0          // Ticks per inch for the middle wheel

// Tracking wheel distances
#define TRACK_WIDTH 14 		  // The distance between left and right wheels (or tracker wheels)
#define MIDDLE_DISTANCE 0     // Distance from middle wheel to the robot turning center

// Sensors
#define IMU_PORT 20                         // Port 0 for disabled
#define ENCODER_PORTS 0, 0, 0                // Port 0 for disabled,
#define EXPANDER_PORT 0                      // Port 0 for disabled
#define ENCODER_TYPE gfr::odom::ENCODER_ADI // The type of encoders

// Movement tuning
#define SLEW_STEP 8          // Smaller number = more slew
#define LINEAR_EXIT_ERROR 1  // default exit distance for linear movements
#define ANGULAR_EXIT_ERROR 1 // default exit distance for angular movements
#define SETTLE_THRESH_LINEAR .5      // amount of linear movement for settling
#define SETTLE_THRESH_ANGULAR 1      // amount of angular movement for settling
#define SETTLE_TIME 250      // amount of time to count as settled
#define LINEAR_KP 1
#define LINEAR_KI 0
#define LINEAR_KD 0
#define TRACKING_KP 60		 // point tracking turning strength
#define ANGULAR_KP 1
#define ANGULAR_KI 0
#define ANGULAR_KD 0
#define MIN_ERROR 5          // Minimum distance to target before angular componenet is disabled
#define LEAD_PCT .6			 // Go-to-pose lead distance ratio (0-1)


// Initializer
inline void init() {

	chassis::init({LEFT_MOTORS}, {RIGHT_MOTORS}, GEARSET, SLEW_STEP, LINEAR_EXIT_ERROR,
	              ANGULAR_EXIT_ERROR, SETTLE_THRESH_LINEAR, SETTLE_THRESH_ANGULAR, SETTLE_TIME);

	odom::init(ODOM_DEBUG, ENCODER_TYPE, {ENCODER_PORTS}, EXPANDER_PORT, IMU_PORT,
	           TRACK_WIDTH, MIDDLE_DISTANCE, TPI,
	           MIDDLE_TPI);

	pid::init(LINEAR_KP, LINEAR_KI, LINEAR_KD, ANGULAR_KP, ANGULAR_KI, ANGULAR_KD, TRACKING_KP, MIN_ERROR, LEAD_PCT);



}

} // namespace arms

#endif
