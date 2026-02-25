#include "mtp.hpp"
#include <list>
#include <vector>
#include "localisation.hpp"
#include "subsystems.hpp"
#include "main.h"
uint16_t movement::g_cost[N];
uint16_t movement::parent[N];
bool movement::closed[N];
movement::SearchNode movement::open_heap[N];
uint8_t movement::da_grid[vertical_nodes][horizontal_nodes];
void movement::add_obstruction(obstruction obs){

    int inflate =
        static_cast<int>(ROBOT_RADIUS_MM / res);

    for(int y = obs.y - inflate; y <= obs.y2 + inflate; y++){
        for(int x = obs.x - inflate; x <= obs.x2 + inflate; x++){

            if(x < 0 || y < 0 || x >= W || y >= H)
                continue;

            da_grid[y][x] = true;
        }
    }
}

int movement::reconstruct_path(int goal_index) {

        int length = 0;
        int current = goal_index;

        while (current != -1 && length < max_path_length) {
            int x, y;
            to_coord(current, x, y);

            current_path[length].x = x;
            current_path[length].y = y;
            current_path[length].find_world(res);

            current = parent[current];
            length++;
        }

        // reverse path
        for (int i = 0; i < length / 2; i++) {
            std::swap(current_path[i],
                    current_path[length - i - 1]);
        }

    return length;
}

std::vector<movement::node> movement::find_path(int sx, int sy, int gx, int gy) {

     heap_size = 0;

    for (int i = 0; i < N; i++) {
        g_cost[i] = INT_MAX;
        parent[i] = -1;
        closed[i] = false;
    }

    int start = to_index(sx, sy);
    int goal  = to_index(gx, gy);

    g_cost[start] = 0;
    heap_push(start, heuristic(sx, sy, gx, gy));

    while (heap_size > 0) {
        int current = heap_pop();
        if (closed[current]) continue;
        closed[current] = true;

        if (current == goal) {

        int length = reconstruct_path(goal);

        if (length == 0)
            return {};

        std::vector<node> merged;
        merged.reserve(length);

        
        merged.push_back(current_path[0]);

        if (length == 1)
            return merged;

        int prev_dx = current_path[1].x - current_path[0].x;
        int prev_dy = current_path[1].y - current_path[0].y;

        for (int i = 2; i < length; i++) {

            int dx = current_path[i].x - current_path[i-1].x;
            int dy = current_path[i].y - current_path[i-1].y;

            if (dx != prev_dx || dy != prev_dy) {
                merged.push_back(current_path[i-1]);
                prev_dx = dx;
                prev_dy = dy;
            }
        }

        merged.push_back(current_path[length - 1]);

        return merged;
    }

        int cx, cy;
        to_coord(current, cx, cy);

        const int dx[8] = { 1,-1, 0, 0, 1, 1,-1,-1 };
        const int dy[8] = { 0, 0, 1,-1, 1,-1, 1,-1 };

        const int move_cost[8] = { 10,10,10,10,14,14,14,14 };

        for (int i = 0; i < 8; i++) {

        int nx = cx + dx[i];
        int ny = cy + dy[i];

        if (nx < 0 || ny < 0 || nx >= W || ny >= H)
            continue;

        if (da_grid[ny][nx])
            continue;

        // Prevent corner cutting
        if (i >= 4) {
            if (da_grid[cy][nx] || da_grid[ny][cx])
                continue;
        }

        int neighbor = to_index(nx, ny);
        if (closed[neighbor]) continue;

        int tentative_g = g_cost[current] + move_cost[i];

        if (tentative_g < g_cost[neighbor]) {
            g_cost[neighbor] = tentative_g;
            parent[neighbor] = current;

            int f = tentative_g + heuristic(nx, ny, gx, gy);
            heap_push(neighbor, f);
        }
        }
    }

    // nochin
    return {};
}
std::vector<movement::node> movement::find_path(start st, int gx, int gy){
    return find_path(st.x , st.y ,gx,gy);
}
std::vector<movement::node> movement::find_path(start st, waypoint way){
    return find_path(st.x , st.y ,way.x,way.y);
}
std::vector<movement::node> movement::find_path(int sx, int sy, waypoint way){
    return find_path(sx , sy ,way.x,way.y);
}

float dist(float x, float y, float x2, float y2){
    return(sqrt((x2-x)*(x2-x) + (y2-y)*(y2-y)));
}
void movement::follow_path(std::vector<node>& path, ez::Drive chassis) {

    for (int i = 0; i < path.size() - 1; i++) {

        float start_x = loco.get_pose().get_x();
        float start_y = loco.get_pose().get_y();
        float current_heading = loco.get_real_sensors().heading_deg;

        float target_x = path[i+1].pos_x;
        float target_y = path[i+1].pos_y;

        float dx = target_x - start_x;
        float dy = target_y - start_y;

        int drive_amount = static_cast<int>(dist(start_x, start_y, target_x, target_y));
        int drive_speed = 70;
        int turn_speed = 90;

        float target_angle = atan2(dy, dx) * 180.0f / M_PI;

        float turn_angle = target_angle - current_heading;
        while (turn_angle > 180) turn_angle -= 360;
        while (turn_angle < -180) turn_angle += 360;

        chassis.pid_turn_set(turn_angle, turn_speed);
        chassis.pid_wait_quick_chain();

        chassis.pid_drive_set(drive_amount, drive_speed);
        chassis.pid_wait_quick_chain();

        loco.set_pose(target_x, target_y, loco.get_real_sensors().heading_deg);
    }
}