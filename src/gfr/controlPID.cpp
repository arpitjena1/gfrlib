

#include <iostream>
#include <math.h>
#include "gfr/controlPID.h"
#include "gfr/util.h"




/**
 * @brief Construct a new PID
 *
 * @param kF feedfoward gain, multiplied by target and added to output. Set 0 if disabled
 * @param kA acceleration gain, limits the change in output. Set 0 if disabled
 * @param kP proportional gain, multiplied by error and added to output
 * @param kI integral gain, multiplied by total error and added to output
 * @param kD derivative gain, multiplied by change in error and added to output
 * @param name name of the PID. Used for logging
 */
gfr::PID::PID(float kF, float kA, float kP, float kI, float kD, std::string pidname) {
    this->kF = kF;
    this->kA = kA;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    
}

/**
 * @brief Set gains
 *
 * @param kF feedfoward gain, multiplied by target and added to output. Set 0 if disabled
 * @param kA acceleration gain, limits the change in output. Set 0 if disabled
 * @param kP proportional gain, multiplied by error and added to output
 * @param kI integral gain, multiplied by total error and added to output
 * @param kD derivative gain, multiplied by change in error and added to output
 */
void gfr::PID::setGains(float kF, float kA, float kP, float kI, float kD) {
    this->kF = kF;
    this->kA = kA;
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
}

/**
 * @brief Set the exit conditions
 *
 * @param largeError
 * @param smallError
 * @param largeTime
 * @param smallTime
 * @param maxTime
 */
void gfr::PID::setExit(float largeError, float smallError, int largeTime, int smallTime, int maxTime) {
    this->largeError = largeError;
    this->smallError = smallError;
    this->largeTime = largeTime;
    this->smallTime = smallTime;
    this->maxTime = maxTime;
}

/**
 * @brief Update the PID
 *
 * @param target the target value
 * @param position the current value
 * @param log whether to check the most recent terminal input for user input. Default is false because logging multiple
 * PIDs could slow down the program.
 * @return float - output
 */
float gfr::PID::update(float target, float position, bool log) {
    // check most recent input if logging is enabled

    // calculate output
    float error = target - position;
    float deltaError = error - prevError;
    float output = kF * target + kP * error + kI * totalError + kD * deltaError;
    if (kA != 0) output = gfr::util::slew(output, prevOutput, kA);
    prevOutput = output;
    prevError = error;
    totalError += error;
    return output;
}

/**
 * @brief Reset the PID
 */
void gfr::PID::reset() {
    prevError = 0;
    totalError = 0;
    prevOutput = 0;
}

/**
 * @brief Check if the PID has settled
 *
 * If the exit conditions have not been set, this function will always return false
 *
 * @return true - the PID has settled
 * @return false - the PID has not settled
 */
bool gfr::PID::settled() {
    if (startTime == 0) { // if maxTime has not been set
        startTime = pros::c::millis();
        return false;
    } else { // check if the PID has settled
        if (pros::c::millis() - startTime > maxTime) return true; // maxTime has been exceeded
        if (std::fabs(prevError) < largeError) { // largeError within range
            if (!largeTimeCounter) largeTimeCounter = pros::c::millis(); // largeTimeCounter has not been set
            else if (pros::c::millis() - largeTimeCounter > largeTime) return true; // largeTime has been exceeded
        }
        if (std::fabs(prevError) < smallError) { // smallError within range
            if (!smallTimeCounter) smallTimeCounter = pros::c::millis(); // smallTimeCounter has not been set
            else if (pros::c::millis() - smallTimeCounter > smallTime) return true; // smallTime has been exceeded
        }
        // if none of the exit conditions have been met
        return false;
    }
}

