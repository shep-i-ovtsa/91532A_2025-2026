#include "tasks.h"
#include <string>
#include "EZ-Template/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "main.h"




void intake_function(void* param){
  while(true){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
      forward_intake.move_velocity(180);
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
      forward_intake.move_velocity(-180);
    } else {
      forward_intake.move_velocity(0);
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      back_roller.move_velocity(180);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      back_roller.move_velocity(-180);
    } else {
      back_roller.move_velocity(0);
    }
    pros::delay(40);

  }
}

void center_score_function(void* param){
  bool open = false;
  while(true){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      open = !open;
    }
    master.set_text(2,0,std::to_string(open));
    center_score.set_value(open);
    pros::delay(100);
  }
}

