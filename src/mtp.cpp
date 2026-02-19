#include "mtp.hpp"

movement::movement(localisation& loc) : loco(loc), pos(loc.get_current_pose()), known_obsticles(loc.get_obsticles()){

}