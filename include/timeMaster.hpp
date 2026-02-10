#ifndef _TIMEMASTER_HPP_
#define _TIMEMASTER_HPP_
enum state {PAUSED, RUNNING, IDLE}; 
class Timer_class{
private:
    double time;
    state timer_state = state::PAUSED;
public:
    double get_time();
    state get_state();
    void start_time();
    void pause_time();
    void reset_time();
    void set_time(int time);

};    
void time_keeper(void* param);
#endif