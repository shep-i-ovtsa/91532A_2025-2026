#include "ratatroller/keybinds.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

// If you are using nlohmann/json, include it here.
// #include <nlohmann/json.hpp>

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
