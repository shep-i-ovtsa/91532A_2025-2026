#include <string>
#include "EZ-Template/util.hpp"
#include "pros/device.hpp"
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
    pros::delay(40);

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
  bool unlock = false;
  bool timer = false;

  enum target {
    BLUE = 120,
    RED  = 35,
    ANY
  };

  target target_current = ANY;   

  partner.clear();
  pros::delay(50);            

  while (true) {

    if (pros::competition::is_autonomous()) {
      pros::delay(50);
      continue;
    }


    if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      target_current = BLUE;
      partner.rumble(".");
    }
    else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      target_current = RED;
      partner.rumble(".");
    }
    else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      target_current = ANY;
      partner.rumble(".");
    }
    else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      unlock = !unlock;
      partner.rumble(".");
    }
    else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      Descore.retract();
    }
    else if (partner.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      timer = !timer;
    }


    switch (target_current) {
      case BLUE: partner.set_text(0, 0, "Target: BLUE"); break;
      case RED:  partner.set_text(0, 0, "Target: RED "); break;
      case ANY:  partner.set_text(0, 0, "Target: ANY  "); break;
    }
    pros::delay(50);
    partner.set_text(1, 0, unlock ? "unlocked" : "locked  ");
    pros::delay(50);
    partner.set_text(2, 0, timer  ? "timer on " : "timer off");

    /* ---------------- Logic ---------------- */

    if (unlock) {

      if (eyes.get_led_pwm() < 75)
        eyes.set_led_pwm(100);

      int hue = eyes.get_hue();

      if (target_current == BLUE) {
        if (hue > BLUE)
          Descore.retract();
      }
      else if (target_current == RED) {
        if (hue < RED)
          Descore.retract();
      }
      else { // ANY
        if (hue < RED || hue > BLUE)
          Descore.retract();
      }

      if (timer && !Descore.is_extended()) {
        pros::delay(2000); 
        Descore.extend();
      }
    }
    else {
      if (eyes.get_led_pwm() > 60)
        eyes.set_led_pwm(0);
    }

    pros::delay(50); 
  }
}
