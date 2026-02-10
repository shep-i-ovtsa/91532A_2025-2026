#include "timeMaster.hpp"
#include "pros/rtos.hpp"

// Static member definitions
double Timer_class::time = 105.0;
state Timer_class::timer_state = state::IDLE;

double Timer_class::get_time() {
    return time;
}

state Timer_class::get_state() {
    return timer_state;
}

void Timer_class::start_time() {
    timer_state = state::RUNNING;
}

void Timer_class::pause_time() {
    timer_state = state::PAUSED;
}

void Timer_class::reset_time() {
    time = 105.0;
    timer_state = state::IDLE;
}

void Timer_class::set_time(double new_time) {
    time = new_time;
}

void time_keeper_proc(void* param) {
    constexpr double tick = 0.1; // seconds

    while (true) {
        if (Timer_class::get_state() == state::RUNNING) {
            Timer_class::set_time(
                Timer_class::get_time() - tick
            );
        }

        pros::delay(
            Timer_class::get_state() == state::RUNNING
                ? (tick * 1000) : 500
        );
    }
}
