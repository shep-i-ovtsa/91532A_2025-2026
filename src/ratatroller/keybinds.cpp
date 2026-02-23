#include "ratatroller/keybinds.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

namespace rat {

namespace {

constexpr std::size_t digital_button_count = 8; //even though i doubt they will add more buttons to the controller. adjust this for more buttons

} 

keybinds::keybinds() { //ADJUST THEESE FOR KEYBINDS
    action_to_button_[to_index(action::NAV_UP)]= button::UP;
    action_to_button_[to_index(action::NAV_DOWN)]= button::DOWN;
    action_to_button_[to_index(action::NAV_LEFT)]= button::LEFT;
    action_to_button_[to_index(action::NAV_RIGHT)]= button::RIGHT;
    action_to_button_[to_index(action::SELECT)]= button::A;
    action_to_button_[to_index(action::BACK)]= button::B;
}


button keybinds::signal(action a) const {
    return action_to_button_[to_index(a)];
}

action keybinds::resolve(button b) const {
    return button_to_action_[to_index(b)];
}

button keybinds::find(pros::controller_digital_e_t raw) const {
    return static_cast<button>(raw);
}
pros::controller_digital_e_t to_pros(button b) {
    switch (b) {
        case button::UP:    return pros::E_CONTROLLER_DIGITAL_UP;
        case button::DOWN:  return pros::E_CONTROLLER_DIGITAL_DOWN;
        case button::LEFT:  return pros::E_CONTROLLER_DIGITAL_LEFT;
        case button::RIGHT: return pros::E_CONTROLLER_DIGITAL_RIGHT;
        case button::A:     return pros::E_CONTROLLER_DIGITAL_A;
        case button::B:     return pros::E_CONTROLLER_DIGITAL_B;
        case button::X:     return pros::E_CONTROLLER_DIGITAL_X;
        case button::Y:     return pros::E_CONTROLLER_DIGITAL_Y;
        default:
            throw std::runtime_error("Unkown");
    }
}
button from_pros(pros::controller_digital_e_t raw) {
    switch (raw) {
        case pros::E_CONTROLLER_DIGITAL_UP:    return button::UP;
        case pros::E_CONTROLLER_DIGITAL_DOWN:  return button::DOWN;
        case pros::E_CONTROLLER_DIGITAL_LEFT:  return button::LEFT;
        case pros::E_CONTROLLER_DIGITAL_RIGHT: return button::RIGHT;
        case pros::E_CONTROLLER_DIGITAL_A:     return button::A;
        case pros::E_CONTROLLER_DIGITAL_B:     return button::B;
        case pros::E_CONTROLLER_DIGITAL_X:     return button::X;
        case pros::E_CONTROLLER_DIGITAL_Y:     return button::Y;
        default:
            throw std::runtime_error("Unknown");
    }
}
void keybinds::rebuild_reverse_map() {
    std::fill(button_to_action_.begin(),
              button_to_action_.end(),
              action::NAV_UP);

    for (std::size_t i = 0; i < action_count; ++i) {
        button b = action_to_button_[i];
        button_to_action_[to_index(b)] =
            static_cast<action>(i);
    }
}

}
