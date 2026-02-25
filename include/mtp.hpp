#ifndef MTP_HPP
#define MTP_HPP

#include "EZ-Template/util.hpp"
#include "localisation.hpp"
#include <array>
#include <cstdint>
#include <climits>
#include <cmath>
#include "subsystems.hpp"
struct obstruction{
public:
    int x;
    int x2;
    int y;
    int y2; //scuare :>
};
struct waypoint{
    int x; 
    int y;
    int theta;
};

struct check_point{
public:
    int left_read;
    int right_read;
    int back_read;
    int front_read;
    float theta; // the angle its required to drive towards at
};
class PoseChecker{
private:
    static constexpr float MAX_SENSOR_RANGE = 4000.0f;
    static constexpr float STEP = 5.0f;

    static float raycast(
        float x_mm,
        float y_mm,
        float theta_deg,
        const uint8_t* grid,
        int W,
        int H,
        float resolution
    ){
        float theta = theta_deg * 3.14159265f / 180.0f;

        float dist = 0.0f;

        while(dist < MAX_SENSOR_RANGE){

            float rx_mm = x_mm + dist * std::cos(theta);
            float ry_mm = y_mm + dist * std::sin(theta);

            int gx = static_cast<int>(rx_mm / resolution);
            int gy = static_cast<int>(ry_mm / resolution);

            if(gx < 0 || gx >= W || gy < 0 || gy >= H)
                return dist;

            if(grid[gy * W + gx] == 1)
                return dist;

            dist += STEP;
        }

        return MAX_SENSOR_RANGE;
    }

    static check_point simulate(
        float x_mm,
        float y_mm,
        float theta_deg,
        const uint8_t* grid,
        int W,
        int H,
        float resolution
    ){
        check_point cp;

        cp.front_read = static_cast<int>(
            raycast(x_mm,y_mm,theta_deg,grid,W,H,resolution)
        );

        cp.back_read = static_cast<int>(
            raycast(x_mm,y_mm,theta_deg+180.0f,grid,W,H,resolution)
        );

        cp.left_read = static_cast<int>(
            raycast(x_mm,y_mm,theta_deg+90.0f,grid,W,H,resolution)
        );

        cp.right_read = static_cast<int>(
            raycast(x_mm,y_mm,theta_deg-90.0f,grid,W,H,resolution)
        );

        cp.theta = theta_deg;
        return cp;
    }

    static int compute_error(
        const check_point& predicted,
        const check_point& actual
    ){
        int e = 0;

        e += std::abs(predicted.front_read - actual.front_read);
        e += std::abs(predicted.back_read  - actual.back_read);
        e += std::abs(predicted.left_read  - actual.left_read);
        e += std::abs(predicted.right_read - actual.right_read);

        return e;
    }

public:
    static void correct_pose(
    localisation& loco,
    const check_point& actual,
    const uint8_t* grid,
    int W,
    int H,
    float resolution)
{
    position& pose = loco.get_pose();

    float base_x     = pose.get_x();
    float base_y     = pose.get_y();
    float base_theta = pose.get_theta();

    float best_x     = base_x;
    float best_y     = base_y;
    float best_theta = base_theta;

    int best_error = INT_MAX;

    const int pos_steps[3] = {50, 25, 5}; //if we cant find something at one resolution, check a higher one
    const int ang_steps[3] = {5, 3, 1};// same thing but for angle, helps with imu bias

    for(int stage = 0; stage < 3; ++stage){

        int pos_step = pos_steps[stage];
        int ang_step = ang_steps[stage];

        for(int dx = -pos_step; dx <= pos_step; dx += pos_step){
            for(int dy = -pos_step; dy <= pos_step; dy += pos_step){
                for(int dt = -ang_step; dt <= ang_step; dt += ang_step){

                    float test_x = best_x + dx;
                    float test_y = best_y + dy;
                    float test_theta = best_theta + dt;

                    check_point predicted =
                        simulate(test_x, test_y, test_theta,
                                 grid, W, H, resolution);

                    int err = compute_error(predicted, actual);

                    if(err < best_error){
                        best_error = err;
                        best_x     = test_x;
                        best_y     = test_y;
                        best_theta = test_theta;
                    }
                }
            }
        }
    }


    if(best_error < 5000){
        pose.set_pose(best_x, best_y, best_theta);
    }
}
};
   
class movement{


public:
    static constexpr float ROBOT_RADIUS_MM = 200.0f; //we imagine the robot as a circle with outward rays
    struct node {
    public:
        int x = 0;
        int y = 0;
        float theta = 0;
        int pos_x = 0;
        int pos_y = 0;

        node() : x(0), y(0), theta(0), pos_x(0), pos_y(0) {}

        node(int x, int y) : x(x), y(y) {
            find_world(res);
        }

        void find_world(float resolution){
            pos_x = static_cast<int>(x * resolution);
            pos_y = static_cast<int>(y * resolution);
        }
    };
    struct start{
    public:
        int x = 0;
        int y = 0;
        float theta = 0;
        position pose;
        start(position pose) : pose(pose){
            x = pose.get_x()/res;
            y = pose.get_y()/res;
            theta = pose.get_theta();
        }
        void update(){
            x = pose.get_x()/res;
            y = pose.get_y()/res;
            theta = pose.get_theta();            
        }
    };
    struct waypoint{
        int x;
        int y;
        float theta;
    };
    struct path{
        std::vector<movement::node>& contents;
        path(std::vector<movement::node>& temp_path) : contents(temp_path){}
    };
    //movement();
    int reconstruct_path(int goal_index);
    std::vector<movement::node> find_path(int sx, int sy, int gx, int gy);
    std::vector<movement::node> find_path(start st, waypoint way);
    std::vector<movement::node> find_path(start st, int gx, int gy);
    std::vector<movement::node> find_path(int sx, int sy, waypoint way);
    static constexpr float FIELD_WIDTH  = 3860.0; //should be self explanetory
    static constexpr float FIELD_HEIGHT = 3860.0;
    static constexpr float res = 25.0; //the resolution we create nodes at
    //! in mms btw

    static constexpr int horizontal_nodes =
        static_cast<int>(FIELD_WIDTH / res);

    static constexpr int vertical_nodes =
        static_cast<int>(FIELD_HEIGHT / res);

    static constexpr int max_path_length = 600;

    void add_obstruction(obstruction obs);

    std::array<node, max_path_length> current_path;

    explicit movement(localisation& l) : loco(l) {}

    void follow_path(std::vector<node>& path , ez::Drive chassis);
private:
 //i think ima go for an A* style alogrhythim that avoids allocating memory often
    static constexpr int W = horizontal_nodes;
    static constexpr int H = vertical_nodes;
    static constexpr int N = W * H; //compress the index for speed
    localisation& loco;
    struct SearchNode {
        int index;
        int f;
    };

    static uint16_t g_cost[N];
    static uint16_t parent[N];
    static bool closed[N];

    static SearchNode open_heap[N];
    int heap_size = 0;
    // 0:free, 1:blocked
    static uint8_t da_grid[vertical_nodes][horizontal_nodes];


    inline int to_index(int x, int y) const {
        return y * W + x;
    }

    inline void to_coord(int index, int& x, int& y) const {
        y = index / W;
        x = index % W;
    }
    inline int heuristic(int x1, int y1, int x2, int y2) const {
        int dx = abs(x1 - x2);
        int dy = abs(y1 - y2);
        return 10 * (dx + dy) + (14 - 20) * std::min(dx, dy);
    }
    void heap_push(int index, int f) {
        int i = heap_size++;
        open_heap[i] = {index, f};

        while (i > 0) {
            int p = (i - 1) / 2;
            if (open_heap[p].f <= open_heap[i].f) break;
            std::swap(open_heap[p], open_heap[i]);
            i = p;
        }
    }
    int heap_pop() {
        int result = open_heap[0].index;
        open_heap[0] = open_heap[--heap_size];

        int i = 0;
        while (true) {
            int l = 2*i + 1;
            int r = 2*i + 2;
            int smallest = i;

            if (l < heap_size && open_heap[l].f < open_heap[smallest].f)
                smallest = l;
            if (r < heap_size && open_heap[r].f < open_heap[smallest].f)
                smallest = r;

            if (smallest == i) break;

            std::swap(open_heap[i], open_heap[smallest]);
            i = smallest;
        }

        return result;
    }

};
#endif

/*
//* 1:get from one point to another
// //* A:split the field into nodes by resoultion
//        -How do we split the field into resolution?
//        -1:We can take the field dimmensions and divide them by res
//s            take those leftover points and turn them into nodes
//    B:store nodes in a list to be iterated over
//        -just use a list and a set of field coordinates depending on res;
//        -remove obscured nodes from the list to avoid including in list

//* C:Drive from node to node
//    feed our nodes to an a* algorythim and feed that back into a movement chain
//    follow the movement chain between nodes to find the fastest route

//* D: repurpose the localisation class to return a set of sensor readings at any angle for a node to cross examine against our current readings to find our error
*/