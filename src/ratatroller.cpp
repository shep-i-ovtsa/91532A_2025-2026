#include "ratatroller.hpp"
#include "subsystems.hpp"

ratatroller::ratatroller(target who){
    chassis.pid_tuner_disable();
    this-> current_target = who;
}