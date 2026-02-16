#include "timeMaster.hpp"
#include "pros/rtos.hpp"

double Timer_class::time = 105.0;
timer_state Timer_class::current_timer_state = timer_state::IDLE;

double Timer_class::get_time() {
    return time;
}

timer_state Timer_class::get_state() {
    return current_timer_state;
}

void Timer_class::start_time() {
    current_timer_state = timer_state::RUNNING;
}

void Timer_class::pause_time() {
    current_timer_state = timer_state::PAUSED;
}

void Timer_class::reset_time() {
    time = 105.0;
    current_timer_state = timer_state::IDLE;
}

void Timer_class::set_time(double new_time) {
    time = new_time;
}

void time_keeper_proc(void* param) {
    constexpr double tick = 0.1; // seconds

    while (true) {
        if (Timer_class::get_state() == timer_state::RUNNING) {
            Timer_class::set_time(
                Timer_class::get_time() - tick
            );
        }

        pros::delay(
            Timer_class::get_state() == timer_state::RUNNING
                ? (tick * 1000) : 500
        );
    }
}
