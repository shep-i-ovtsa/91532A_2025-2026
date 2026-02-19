#include "mtp.hpp"
#include <cmath>
#include "subsystems.hpp"

// ==========================================================
// Strategy Implementations
// ==========================================================

float forward_approach::compute_heading(
    float cx, float cy,
    float tx, float ty)
{
    return atan2f(ty - cy, tx - cx) * 180.0f / M_PI;
}

float backward_approach::compute_heading(
    float cx, float cy,
    float tx, float ty)
{
    float angle = atan2f(ty - cy, tx - cx) * 180.0f / M_PI;
    return fmodf(angle + 180.0f, 360.0f);
}

// ==========================================================
// Movement Constructor
// ==========================================================

movement::movement(localisation& loc) : loco(loc){
}

// ==========================================================
// Public Movement Functions
// ==========================================================

void movement::move_to(const objective& target, approach_strategy& strategy){
    // Generate path
    current_path = create_path(target, strategy);

    // Follow each link in order
    for (const auto& node : current_path) {
        follow_link(node);
    }

    // Optional final orientation correction
    if (target.theta.has_value()) {
        // TODO:
        // chassis.set_turn_target(target.theta.value());
        // chassis.wait_drive();
    }
}

void movement::move_to_interact(const objective& target, approach_strategy& strategy)
{
    move_to(target, strategy);

    if (target.interact) {
        target.interact();
    }
}

void movement::move_to(int x, int y, int theta)
{
    objective temp;
    temp.x = x;
    temp.y = y;
    temp.theta = theta;

    forward_approach forward;
    move_to(temp, forward);
}

std::vector<link> movement::create_path(
    const objective& target,
    approach_strategy& strategy)
{
    std::vector<link> path;

    position& pose = loco.get_current_pose();

    float cx = pose.get_x();
    float cy = pose.get_y();

    float dx = target.x - cx;
    float dy = target.y - cy;

    float distance = compute_distance(cx, cy, target.x, target.y);

    const float SEGMENT_LENGTH = 300.0f; 

    int segments = static_cast<int>(distance / SEGMENT_LENGTH);
    if (segments < 1) segments = 1;

    for (int i = 1; i <= segments; i++) {
        float ratio = static_cast<float>(i) / segments;

        link node;
        node.x_mm = cx + dx * ratio;
        node.y_mm = cy + dy * ratio;

        node.theta = strategy.compute_heading(
            cx, cy,
            node.x_mm, node.y_mm
        );

        path.push_back(node);
    }

    return path;
}

void movement::follow_link(const link& next)
{
    const float ARRIVAL_THRESHOLD = 75.0f; // mm tolerance

    while (true) {

        position& pose = loco.get_current_pose();

        float cx = pose.get_x();
        float cy = pose.get_y();

        float distance = compute_distance(
            cx, cy,
            next.x_mm, next.y_mm
        );

        if (distance < ARRIVAL_THRESHOLD)
            break;

        float heading = next.theta;

        obstacle_adjust(heading, cx, cy);
        //todo decide wtf i want the movement to be
        //* for now just
        chassis.pid_turn_chain_constant_set(heading);
        chassis.pid_wait_quick_chain();
        chassis.pid_drive_set(distance,speed);
    }
}

float movement::compute_distance(float x1, float y1, float x2, float y2){
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

void movement::obstacle_adjust(float& heading,float cx, float cy){

}
