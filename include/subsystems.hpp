#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor forward_intake(10);
inline pros::Motor back_roller(9);
inline pros::adi::Pneumatics hammerHead('A', false);
inline pros::adi::Pneumatics Descore('B', false);
inline pros::Imu imu(4);

// inline pros::adi::DigitalIn limit_switch('A');