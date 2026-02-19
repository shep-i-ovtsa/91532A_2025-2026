#ifndef MTP_HPP
#define MTP_HPP

#include <vector>
#include <optional>
#include <string>
#include "localisation.hpp"


struct objective {
    float x;
    float y;
    std::optional<float> theta;
    std::string name;

    void (*interact)() = nullptr;

    void set_interact_behavior(void (*fn)()) {
        interact = fn;
    }
};
struct link{
    float x_mm;
    float y_mm;
    std::optional<float> theta;
};

class approach_strategy {
public:
    virtual float compute_heading( float current_x, float current_y, float target_x, float target_y) = 0;

    virtual ~approach_strategy() = default; //c++'s weird version of an abstract class
};
class forward_approach : public approach_strategy { //in java terms pretty much this is its version of public class forward_approach extends approach_strategy
public:
    float compute_heading(float cx, float cy, float tx, float ty) override; //! @overide compute_heading();
};
class backward_approach :public approach_strategy {
public:
    float compute_heading(float cx, float cy, float tx, float ty) override; //? lowk not used to using c++s abstract methods but i think im translating right
};

class movement {
public:
    movement(localisation& loc);

    void move_to(const objective& target, approach_strategy& strategy); //only move to the target in the specified way

    void move_to_interact(const objective& target, approach_strategy& strategy);//move to the target in its specified way but call the interact function

    void move_to(int x, int y, int theta); //move to the coordinates

private:
    std::vector<link> create_path(const objective& target, approach_strategy& strategy);
    void follow_link(const link& next);
    localisation& loco;
    std::vector<link> current_path;

    float compute_distance(float x1, float y1, float x2, float y2);

    void obstacle_adjust(float& heading, float cx, float cy);
    //todo find a convenient way to set speed
};

#endif