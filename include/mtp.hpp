#ifndef MTP_HPP
#define MTP_HPP

#include <vector>
#include "localisation.hpp"
class movement{
public:
    movement(localisation& loc);
    void move_to();
private:
    std::vector<obsticle>& known_obsticles;
    localisation& loco;
    position& pos;
};


#endif