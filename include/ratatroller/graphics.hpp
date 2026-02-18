
#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include "keybinds.hpp"
namespace rat {
class screen {
public:
    screen();
    virtual void handle_action(action a) = 0;
    
}; //apperently this is c++'s weird version of an astract class??????/ im gonna jump bro fml im not even done yet >.< 


}
#endif