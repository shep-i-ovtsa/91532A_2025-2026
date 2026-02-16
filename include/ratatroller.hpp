#ifndef RATATROLLER_HPP
#define RATATROLLER_HPP
#include <string>
#include "pros/misc.h"
#include <functional>
#include <array>
#include <vector>
enum class target{
    MASTER = pros::E_CONTROLLER_MASTER,
    PARTNER = pros::E_CONTROLLER_PARTNER
};
enum class keybind{
    KEY_UP = pros::E_CONTROLLER_DIGITAL_UP,
    KEY_DOWN = pros::E_CONTROLLER_DIGITAL_DOWN,
    KEY_LEFT = pros::E_CONTROLLER_DIGITAL_LEFT,
    KEY_RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT,
    A = pros::E_CONTROLLER_DIGITAL_A,
    B = pros::E_CONTROLLER_DIGITAL_B,
    X = pros::E_CONTROLLER_DIGITAL_X,
    Y = pros::E_CONTROLLER_DIGITAL_Y
};
class ratatroller{



public:


    struct menu_option{ //? prototype menu option object
    public:
        void set_behavior_select( std::function<void()> fn);
        void set_behavior_hover( std::function<void()> fn);
        void set_text(const std::string& text);
        std::string get_id();
        menu_option(const std::string& id);
        void select();
        void hover();
     private:
        std::function<void()> on_select;
        std::function<void()> on_hover;
        bool is_hovered = false;
        std::string id;
        std::string text;

    };



    static void controller_task(void* param);

    class menu_panel{ //? decided to focus on menu for now
    public:
        void display_menu();
        menu_option add_option(std::string id, std::string text);
    private:
        std::vector<menu_option> options;
    };
    ratatroller(target who);



private:
    target current_target;
    keybind NAV_FORWARD = keybind::KEY_RIGHT;
    keybind NAV_BACKWARD = keybind::KEY_LEFT;
    keybind NAV_UP = keybind::KEY_UP;
    keybind NAV_DOWN = keybind::KEY_DOWN;
    keybind SELECT = keybind::A;
    keybind RETURN = keybind::B;
};
#endif