#pragma once
#ifndef TIME_MASTER_HPP
#define TIME_MASTER_HPP
#include "states.hpp"

class Timer_class {
public:
    static double get_time();
    static timer_state get_state();

    static void start_time();
    static void pause_time();
    static void reset_time();
    static void set_time(double time);

private:
    static double time;
    static timer_state timer_state;
};

void time_keeper_proc(void* param);
#endif