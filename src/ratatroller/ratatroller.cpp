#include "ratatroller/ratatroller.hpp"
#include "pros/rtos.hpp"
#include "keybinds.hpp"
using namespace rat;

ratatroller::ratatroller(target who) 
    : current_target(who), 
      current_controller(
          who == target::MASTER ? pros::E_CONTROLLER_MASTER 
                                : pros::E_CONTROLLER_PARTNER) 
{
    //start polling asap and we pass our own instance so it dosent crash tf out
    pros::Task ratatroller_task(ratatroller::controller_task, this);
}

void ratatroller::push_screen(std::unique_ptr<screen> s) {
    screen_stack.push_back(std::move(s)); 
}

void ratatroller::push_screen(const std::shared_ptr<screen>& s) {
    screen_stack.push_back(s);
}
//we use this to well...pop a screen...pretty self explanetory
void ratatroller::pop_screen() {
    if (!screen_stack.empty())
        screen_stack.pop_back();
}

//! IM SO FUCKING TIREDDD AUGHHHHHH AND THERES STILL SO MUCH LEFT TO DO T_T
void ratatroller::controller_task(void* param) {
    ratatroller* self = static_cast<ratatroller*> (param); //? i just let the clanker fix this one...no clue why it works but it does ig

    while (true) {
        int size = self -> screen_stack.size();
        if (!self->screen_stack.empty()) {//if our screen stack isint empty

            for (size_t i = 0; i < static_cast<size_t>(action::COUNT); ++i) {//for every action in our list
                action a = static_cast<action>(i); // a is equal to the action were indexing rn
                button b = self->binds.signal(a);
                pros::controller_digital_e_t raw_button = binds.to_pros(b); //raw buttons is equal to whatever button is being pressed in the current moment
                if (self->current_controller.get_digital_new_press(raw_button)) { //get the raw button from our selected controller
                    self->screen_stack[size -1]->handle_action(a); //pass the decoded action to the focused screen
                }
            }            
            self -> screen_stack[size -1]->display_screen();
        }

        pros::delay(100);
    }
}
pros::Controller& ratatroller::get_controller(){
    return(current_controller);
}