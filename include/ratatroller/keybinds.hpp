#ifndef RATATROLLER_KEYBINDS_HPP
#define RATATROLLER_KEYBINDS_HPP

#include "pros/misc.h"
#include "pros/misc.hpp"

#include <array>
#include <string>
#include <cstdint>
#include <stdexcept>

namespace rat {

enum class action : uint8_t { //add actions here
    NAV_UP = 0,
    NAV_DOWN = 1,
    NAV_LEFT = 2,
    NAV_RIGHT = 3,
    SELECT = 4,
    BACK = 5,
    COUNT = 6 //amount of enums
};

enum class button : uint8_t { 
    A     = pros::E_CONTROLLER_DIGITAL_A,
    B     = pros::E_CONTROLLER_DIGITAL_B,
    X     = pros::E_CONTROLLER_DIGITAL_X,
    Y     = pros::E_CONTROLLER_DIGITAL_Y,
    LEFT  = pros::E_CONTROLLER_DIGITAL_LEFT,
    DOWN  = pros::E_CONTROLLER_DIGITAL_DOWN,
    UP    = pros::E_CONTROLLER_DIGITAL_UP,
    RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT
};

class keybinds {
public:
    keybinds(); 
    button signal(action a) const;
    action resolve(button b) const;
    button find(pros::controller_digital_e_t raw) const;
    pros::controller_digital_e_t to_pros(button b);
    button from_pros(pros::controller_digital_e_t raw);
private:
    static constexpr std::size_t action_count =
        static_cast<std::size_t>(action::COUNT);

    using action_map = std::array<button, action_count>;
    using reverse_map = std::array<action, 8>; 

    action_map  action_to_button_{};
    reverse_map button_to_action_{};

    static constexpr std::size_t to_index(action a) { 
        return static_cast<std::size_t>(a);
    }

    static constexpr std::size_t to_index(button b) {
        return static_cast<std::size_t>(
            static_cast<pros::controller_digital_e_t>(b)
        );
    }

    void rebuild_reverse_map();
};

} 

#endif 
