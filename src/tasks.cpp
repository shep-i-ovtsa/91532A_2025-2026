#include "tasks.h"
#include "EZ-Template/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

void matchloader_function(void* param){
  while(true){
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
      hammerHead.set_value(true);
    }
    pros::delay(100); 

    }
}