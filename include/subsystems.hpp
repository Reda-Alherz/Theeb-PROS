#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::MotorGroup intake({1, -15});
inline pros::MotorGroup right1({-19, -18, -17, -13});
inline pros::MotorGroup left1({11, 12, 14, 16});
// inline pros::adi::DigitalIn limit_switch('A');