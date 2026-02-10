#include "states.hpp"

drive_state states::drive() {
    return current_drive_state;
}

position_state states::position() {
    return current_position_state;
}

status_state states::status() {
    return current_status;
}

void states::set_drive(drive_state new_state) {
    current_drive_state = new_state;
}

void states::set_position(position_state new_state) {
    current_position_state = new_state;
}

void states::set_status(status_state new_state) {
    current_status = new_state;
}

