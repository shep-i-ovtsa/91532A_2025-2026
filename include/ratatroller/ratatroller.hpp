#ifndef RATATROLLER_HPP
#define RATATROLLER_HPP
#pragma once
#include <memory>
#include <string>
#include <array>
#include <vector>
#include "pros/misc.hpp"
#include "keybinds.hpp"
#include "graphics.hpp"

namespace rat {
enum class target {
    MASTER = 0,
    PARTNER = 1
};

class ratatroller {
public:
    static void controller_task(void* param);
    void push_screen(std::unique_ptr<screen> screen); 
    void push_screen(const std::shared_ptr<screen>& screen); 
    void pop_screen();
    ratatroller(target who);
    pros::Controller& get_controller();
private:

    target current_target;
    pros::Controller current_controller;
    static keybinds binds;
    std::vector<std::shared_ptr<screen>> screen_stack; //we need to use unique pointers so that if the screen object gets
     // realocated, we dont loose access to it


};

} 
#endif
