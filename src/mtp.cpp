#include "mtp.hpp"
#include "localisation.hpp"
#include "subsystems.hpp"
std::vector<obsticle> temp {};
std::vector<obsticle>& movement::known_obsticles = temp; //idk man

movement::movement(localisation& loc) : loco(loc), pos(loc.get_current_pose()){
    known_obsticles = loc.get_obsticles();
}