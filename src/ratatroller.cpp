#include "ratatroller.hpp"

ratatroller::menu_option::menu_option(const std::string& id){}

void ratatroller::menu_option::set_behavior_select(std::function<void()> fn){}

void ratatroller::menu_option::set_behavior_hover(std::function<void()> fn){}

void ratatroller::menu_option::set_text(const std::string& text){}

std::string ratatroller::menu_option::get_id(){}

void ratatroller::menu_option::select(){}

void ratatroller::menu_option::hover(){}

void ratatroller::menu_panel::display_menu(){}

ratatroller::menu_option ratatroller::menu_panel::add_option(std::string id, std::string text){}

ratatroller::ratatroller(target who){}

void ratatroller::controller_task(void* param){}