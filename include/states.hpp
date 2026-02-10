// todo create a global state class that monitors the robots states, and provides a simple interface for "ezconfigs" in processes

#ifndef STATES_HPP
#define STATES_HPP
enum class timer_state {
    IDLE,
    RUNNING,
    PAUSED
};
enum class status_state {
    ENABLED,
    DISABLED
};
enum class drive_state{
    DRIVER_CONTROL,
    AUTONOMOUS,
    PRACTICE,
};
enum class position_state{
    FLIPPING,
    LEVEL,
    TITANIC
};


class states {
public:
    drive_state drive();
    void set_drive(drive_state new_state);

    position_state position();
    void set_position(position_state new_state);

    status_state status();
    void set_status(status_state new_state);

private:
    drive_state current_drive_state = drive_state::PRACTICE;
    position_state current_position_state = position_state::LEVEL;
    status_state current_status=status_state::DISABLED;
};

#endif