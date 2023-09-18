#include <cmath>
#include <vector>
#include <string>
#include "pros/misc.hpp"
#include "gfr/asset.h"
#include "gfr/api.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}