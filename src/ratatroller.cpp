#include "ratatroller.hpp"
#include <functional>
#include "pros/misc.h"
#include "pros/misc.hpp"

ratatroller::menu_option::menu_option(const std::string& id){
    this -> id = id;

}

void ratatroller::menu_option::set_behavior_select(std::function<void()> fn){
    this -> on_select = fn;
}

void ratatroller::menu_option::set_behavior_hover(std::function<void()> fn){
        this -> on_hover = fn;
}

void ratatroller::menu_option::set_text(const std::string& text){
    this-> text = text;
}

std::string ratatroller::menu_option::get_id(){
    return (this -> id);
}

void ratatroller::menu_option::select(){
    this -> on_select();
}

void ratatroller::menu_option::hover(){
    this -> on_hover();
}

void ratatroller::menu_panel::display_menu(){}

ratatroller::menu_option ratatroller::menu_panel::add_option(std::string id, std::string text){
    menu_option new_option(id);
    new_option.set_text(text);
    return(new_option);
}

ratatroller::ratatroller(target who){
    this -> current_target = who;
    current_controller = pros::E_CONTROLLER_MASTER;   
}

void ratatroller::controller_task(void* param){}