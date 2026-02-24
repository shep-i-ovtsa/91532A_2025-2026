#include "tasks.h"
#include <compare>
#include <string>
#include "EZ-Template/util.hpp"
#include "localisation.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

bool flip  = false;
void matchloader_function(void* param){
  while(true){
    pros::delay(10000);

    }
}

void flip_detection_function(void* param){
  double accel_gs = 0.0;
  double pitch = 0.0;

  while(true){
    pitch = imu.get_pitch();
    accel_gs = imu.get_accel().z;

    float g_norm = clamp(accel_gs / 1.0, 0.0, 1.0);
    float tip_threshold = 40.0 - 11.5 * g_norm;

    if (pitch > tip_threshold) {
      flip = true;
    }
    else if (pitch < tip_threshold - 8) {   
      flip = false;
    }

    pros::delay(pros::competition::is_disabled() ? 400 : 40);
  }
}


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
      if(!center_score.is_extended()){
        center_score.extend();
      }
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      back_roller.move_velocity(-180);
      if(!center_score.is_extended()){
        center_score.extend();
      }
    } else {
      back_roller.move_velocity(0);
    }
    pros::delay(pros::competition::is_disabled() ? 400 : 100);

  }
}

void descore_function(void* param){
  while(true){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      Descore.set_value(true);   
      master.rumble("."); 
    }
    else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
       Descore.set_value(false);
      master.rumble(".");

    }
    pros::delay(pros::competition::is_disabled() ? 400 : 60);
  }
}

void center_score_function(void* param){
  bool open = false;
  while(true){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      open = !open;
      master.rumble(".");
    }
    master.set_text(2,0,std::to_string(open));
    center_score.set_value(open);
    pros::delay(100);
  }
}

void anti_descore(void* param) {

}
