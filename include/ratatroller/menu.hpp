#ifndef MENU_HPP
#define MENU_HPP

#include "ratatroller.hpp"
#include <vector>
struct menu_option{ //? prototype menu option object
    public:
        void set_behavior_select(void (*fn)());
        void set_behavior_hover( void (*fn)());
        void set_text(const std::string& text);
        std::string get_id();
        menu_option(const std::string& id);
        void select();
        void hover();
     private:
        void (*on_select)();
        void (*on_hover)();
       bool is_hovered = false;
        std::string id;
        std::string text;

};

    class menu_panel{ //? decided to focus on menu for now
    public:
        void display_menu();
        menu_option& add_option(std::string id, std::string text);
    private:
        std::vector<menu_option> options;
    };

#endif