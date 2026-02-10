#include "timeMaster.hpp"
#include "EZ-Template/util.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
class Timer_class::Timer_class{
    Timer_class::time = 105.0;
    Timer_class::timer_state = state::IDLE;

    double Timer_class::get_time(){
        return(Timer_class::time);
    }
    state Timer_class::get_state(){
        return(Timer_class::timer_state);
    }
    void Timer_class::start_time(){
        Timer_class::timer_state = state::RUNNING;
    }
    void Timer_class::pause_time(){
        Timer_class::timer_state = state::PAUSED;
    }
    void Timer_class::reset_time(){
        Timer_class::time = 105.0;
    }
    void Timer_class::set_time(int time){
        Timer_class::time = time;
    }
};
void time_keeper_proc(void* param){
    double time_count = 0.1;
    while(true){
        if( == state::RUNNING){
            Timer_class::time -= time_count;
        }
        pros::delay(Timer_class::timer_state == state::PAUSED || Timer_class::timer_state == state::IDLE ? 500 : time_count * 1000);
    }
}