#ifndef _GFR_PID_H_
#define _GFR_PID_H_

#include <array>
#include "point.h"

namespace gfr::pid {

// pid mode enums
extern int mode;
#define DISABLE 0
#define TRANSLATIONAL 1
#define ANGULAR 2

// pid constants
extern double linearKP;
extern double linearKI;
extern double linearKD;
extern double angularKP;
extern double angularKI;
extern double angularKD;
extern double trackingKP;
extern double minError;

// integral
extern double in_lin;
extern double in_ang;

// targets
extern double angularTarget;
extern Point pointTarget;

// flags
extern bool thru;
extern bool reverse;

extern bool canReverse;

// pid functions
std::array<double, 2> translational();
std::array<double, 2> angular();

// initializer
void init(double linearKP, double linearKI, double linearKD, double angularKP,
          double angularKI, double angularKD, double trackingKP, double minError, double leadPct);

} // namespace gfr::pid

#endif
