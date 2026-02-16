#ifndef RATATROLLER_HPP
#define RATATROLLER_HPP

#include <memory>
#include <string>
#include <array>
#include <vector>
#include "pros/misc.hpp"
#include "keybinds.hpp"
#include "graphics.hpp"

namespace rat {
enum class target {
    MASTER,
    PARTNER
};

class ratatroller {
public:
    static void controller_task(void* param);
    void push_screen(std::unique_ptr<screen> screen); 
    void pop_screen();
    ratatroller(target who);
private:

    target current_target;
    pros::Controller current_controller;
    keybinds binds;
    std::vector<std::unique_ptr<screen>> screen_stack; //we need to use unique pointers so that if the screen object gets
     // realocated, we dont loose access to it


};

} 

#endif
