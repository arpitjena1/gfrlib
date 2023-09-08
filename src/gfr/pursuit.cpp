

// The implementation below is mostly based off of
// the document written by Dawgma
// Here is a link to the original document
// https://www.chiefdelphi.com/uploads/default/original/3X/b/e/be0e06de00e07db66f97686505c3f4dde2e332dc.pdf

#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "gfr/asset.h"
#include "gfr/api.h"

/**
 * @brief function that returns elements in a file line, separated by a delimeter
 *
 * @param input the raw string
 * @param delimeter string separating the elements in the line
 * @return std::vector<std::string> array of elements read from the file
 */
std::vector<std::string> readElement(const std::string& input, std::string delimiter) {
    std::string token;
    std::string s = input;
    std::vector<std::string> output;
    size_t pos = 0;

    // main loop
    while ((pos = s.find(delimiter)) != std::string::npos) { // while there are still delimiters in the string
        token = s.substr(0, pos); // processed substring
        output.push_back(token);
        s.erase(0, pos + delimiter.length()); // remove the read substring
    }

    output.push_back(s); // add the last element to the returned string

    return output;
}

/**
 * @brief Get a path from the sd card
 *
 * @param filePath The file to read from
 * @return std::vector<gfr::Pose> vector of points on the path
 */
std::vector<gfr::Pose> getData(asset path) {
    std::vector<gfr::Pose> robotPath;
    std::string line;
    std::vector<std::string> pointInput;
    gfr::Pose pathPoint(0, 0, 0);

    // format data from the asset
    std::string data(reinterpret_cast<char*>(path.buf), path.size);
    std::vector<std::string> dataLines = readElement(data, "\n");

    // read the points until 'endData' is read
    for (std::string line : dataLines) {
        if (line == "endData") break;
        pointInput = readElement(line, ", "); // parse line
        pathPoint.x = std::stof(pointInput.at(0)); // x position
        pathPoint.y = std::stof(pointInput.at(1)); // y position
        pathPoint.theta = std::stof(pointInput.at(2)); // velocity
        robotPath.push_back(pathPoint); // save data
    }

    return robotPath;
}

/**
 * @brief find the closest point on the path to the robot
 *
 * @param pose the current pose of the robot
 * @param path the path to follow
 * @return int index to the closest point
 */
int findClosest(gfr::Pose pose, std::vector<gfr::Pose> path) {
    int closestPoint;
    float closestDist = 1000000;
    float dist;

    // loop through all path points
    for (int i = 0; i < path.size(); i++) {
        dist = pose.distance(path.at(i));
        if (dist < closestDist) { // new closest point
            closestDist = dist;
            closestPoint = i;
        }
    }

    return closestPoint;
}

/**
 * @brief Function that finds the intersection point between a circle and a line
 *
 * @param p1 start point of the line
 * @param p2 end point of the line
 * @param pos position of the robot
 * @param path the path to follow
 * @return float how far along the line the
 */
float circleIntersect(gfr::Pose p1, gfr::Pose p2, gfr::Pose pose, float lookaheadDist) {
    // calculations
    // uses the quadratic formula to calculate intersection points
    gfr::Pose d = p2 - p1;
    gfr::Pose f = p1 - pose;
    float a = d * d;
    float b = 2 * (f * d);
    float c = (f * f) - lookaheadDist * lookaheadDist;
    float discriminant = b * b - 4 * a * c;

    // if a possible intersection was found
    if (discriminant >= 0) {
        discriminant = sqrt(discriminant);
        float t1 = (-b - discriminant) / (2 * a);
        float t2 = (-b + discriminant) / (2 * a);

        // prioritize further down the path
        if (t2 >= 0 && t2 <= 1) return t2;
        else if (t1 >= 0 && t1 <= 1) return t1;
    }

    // no intersection found
    return -1;
}

/**
 * @brief returns the lookahead point
 *
 * @param lastLookaheadIndex - the index of the last lookahead point
 * @param lastLookahead - the last lookahead point
 * @param pos - the current position of the robot
 * @param path - the path to follow
 * @param forward - whether to go forward (true) or backwards (false)
 */
gfr::Pose lookaheadPoint(gfr::Pose lastLookahead, gfr::Pose pose, std::vector<gfr::Pose> path,
                            float lookaheadDist) {
    // initialize variables
    gfr::Pose lookahead = lastLookahead;
    double t;

    // find the furthest lookahead point on the path
    for (int i = 0; i < path.size() - 1; i++) {
        t = circleIntersect(path.at(i), path.at(i + 1), pose, lookaheadDist);
        if (t != -1 && i >= lastLookahead.theta) { // new lookahead point found
            lookahead = path.at(i).lerp(path.at(i + 1), t);
            lookahead.theta = i;
        }
    }

    return lookahead;
}

/**
 * @brief Get the curvature of a circle that intersects the robot and the lookahead point
 *
 * @param pos the position of the robot
 * @param heading the heading of the robot
 * @param lookahead the lookahead point
 * @return double curvature
 */
double findLookaheadCurvature(gfr::Pose pose, double heading, gfr::Pose lookahead) {
    // calculate whether the robot is on the left or right side of the circle
    double side = gfr::sgn(std::sin(heading) * (lookahead.x - pose.x) - std::cos(heading) * (lookahead.y - pose.y));
    // calculate center point and radius
    double a = -std::tan(heading);
    double c = std::tan(heading) * pose.x - pose.y;
    double x = std::fabs(a * lookahead.x + lookahead.y + c) / std::sqrt((a * a) + 1);
    double d = std::hypot(lookahead.x - pose.x, lookahead.y - pose.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

/**
 * @brief Move the chassis along a path
 *
 * @param path the filename of the path to follow
 * @param timeout the maximum time the robot can spend moving
 * @param lookahead the lookahead distance. Units in inches. Larger values will make the robot move faster but will
 * follow the path less accurately
 * @param reverse whether the robot should follow the path in reverse. false by default
 * @param maxSpeed the maximum speed the robot can move at
 * @param log whether the chassis should log the path on a log file. false by default.
 */
void gfr::chassis::follow(asset path, int timeout, float lookahead, bool reverse, float maxSpeed, bool log) {
    std::vector<gfr::Pose> pathPoints = getData(path); // get list of path points
    Pose pose(0, 0, 0);
    Pose lookaheadPose(0, 0, 0);
    Pose lastLookahead = pathPoints.at(0);
    lastLookahead.theta = 0;
    double curvature;
    float targetVel;
    float prevLeftVel = 0;
    float prevRightVel = 0;
    int closestPoint;
    float leftInput = 0;
    float rightInput = 0;
    int compState = pros::competition::get_status();

    // loop until the robot is within the end tolerance
    for (int i = 0; i < timeout / 10 && pros::competition::get_status() == compState; i++) {
        // get the current position of the robot
        pose = gfr::odom::getPose(true);
        if (reverse) pose.theta -= M_PI;

        // find the closest point on the path to the robot
        closestPoint = findClosest(pose, pathPoints);
        // if the robot is at the end of the path, then stop
        if (pathPoints.at(closestPoint).theta == 0) break;

        // find the lookahead point
        lookaheadPose = lookaheadPoint(lastLookahead, pose, pathPoints, lookahead);
        lastLookahead = lookaheadPose; // update last lookahead position

        // get the curvature of the arc between the robot and the lookahead point
        double curvatureHeading = M_PI / 2 - pose.theta;
        curvature = findLookaheadCurvature(pose, curvatureHeading, lookaheadPose);

        // get the target velocity of the robot
        targetVel = pathPoints.at(closestPoint).theta;

        // calculate target left and right velocities
        float targetLeftVel = targetVel * (2 + curvature * TRACK_WIDTH) / 2;
        float targetRightVel = targetVel * (2 - curvature * TRACK_WIDTH) / 2;

        // ratio the speeds to respect the max speed
        float ratio = std::max(std::fabs(targetLeftVel), std::fabs(targetRightVel)) / maxSpeed;
        if (ratio > 1) {
            targetLeftVel /= ratio;
            targetRightVel /= ratio;
        }

        // update previous velocities
        prevLeftVel = targetLeftVel;
        prevRightVel = targetRightVel;

        // move the drivetrain
        if (reverse) {
           leftMotors->move(-targetRightVel);
           rightMotors->move(-targetLeftVel);
        } else {
            leftMotors->move(targetLeftVel);
            rightMotors->move(targetRightVel);
        }

        pros::delay(10);
    }

    // stop the robot
    leftMotors->move(0);
    rightMotors->move(0);
}
