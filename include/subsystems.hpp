#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"

extern Drive chassis;
inline pros::Controller partner(pros::E_CONTROLLER_PARTNER);
// Your motors, sensors, etc. should go here.  Below are examples
inline pros::Motor forward_intake(10);
inline pros::Motor back_roller(9);
inline pros::adi::Pneumatics hammerHead('A', false);
inline pros::adi::Pneumatics Descore('B', false);
inline pros::Imu imu(4);
inline pros::Distance back_sensor(5);
inline pros::Distance left_sensor(3);
inline pros::Distance right_sensor(6);
inline pros::Distance front_sensor(2);

// inline pros::adi::DigitalIn limit_switch('A');