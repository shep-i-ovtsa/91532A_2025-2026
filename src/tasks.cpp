#include "tasks.h"
#include "EZ-Template/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "main.h"

volatile bool flip  = false;
void matchloader_function(void* param){
  while(true){
    if(!flip){
      if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        hammerHead.set_value(true);
      } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
        hammerHead.set_value(false);
      }
    } else {
      hammerHead.set_value(true);
    }
    pros::delay(pros::competition::is_disabled() ? 500 : 100);

    }
}
void flip_detection_function(void* param){
  bool deadswitch = true;
  double accel_gs = 0.0;
  double pitch = 0.0;
  while(true){
    pitch = imu.get_pitch();
    accel_gs = imu.get_accel().z;
    float g_norm = clamp(accel_gs / 1.0, 0.0, 1.0); //yes
    float tip_threshold = 40.0 - 11.5 * g_norm;

    if (deadswitch) {
        if (pitch > tip_threshold) {
          flip = true;
        }
        else if (pitch < tip_threshold - 8) {   
          flip = false;
        }
    }

    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
      deadswitch = !deadswitch;
    }
    pros::delay(pros::competition::is_disabled() ? 500 : 40);
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
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      back_roller.move_velocity(-180);
    } else {
      back_roller.move_velocity(0);
    }
    pros::delay(pros::competition::is_disabled() ? 500 : 100);

  }
}

void descore_function(void* param){
  while(true){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      Descore.set_value(true);    }
    else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
       Descore.set_value(false);
    }
    pros::delay(pros::competition::is_disabled() ? 500 : 100);
  }
}

