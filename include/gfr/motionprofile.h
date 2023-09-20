#include "gfr/api.h"
#include <vector>
#include <cmath>


class MotionProfile {
public:
  std::vector<gfr::Pose>* path;
  float velConstraint;
  float startingTheta;
  float accel;
  float deaccel;
  float d;
  float d1, d2, d3;
  float topSpeed;

  void CalculateVelocities();
  void DetermineTopSpeed();
  void DetermineTimes();
  MotionProfile(std::vector<gfr::Pose>* path, float velConstraint, float accelConstraint, float deaccelConstraint, float startingTheta = 0);
  void DetermineDistances();

};