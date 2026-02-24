#include "mtp.hpp"
#include <list>
#include "localisation.hpp"
#include "subsystems.hpp"

void movement::add_obstruction(obstruction obs){

    int inflate =
        static_cast<int>(ROBOT_RADIUS_MM / res);

    for(int y = obs.y - inflate; y <= obs.y2 + inflate; y++){
        for(int x = obs.x - inflate; x <= obs.x2 + inflate; x++){

            if(x < 0 || y < 0 || x >= W || y >= H)
                continue;

            da_grid[y][x] = 1;
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
            // Reconstruct path into a vector
            std::vector<node> path;
            int cur = goal;
            while (cur != -1) {
                int x, y;
                to_coord(cur, x, y);
                node n(x, y);
                path.push_back(n);
                cur = parent[cur];
            }
            // reverse to start → goal
            std::reverse(path.begin(), path.end());
            return path;
        }

        int cx, cy;
        to_coord(current, cx, cy);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (da_grid[ny][nx] == 1) continue;

            int neighbor = to_index(nx, ny);
            if (closed[neighbor]) continue;

            int tentative_g = g_cost[current] + 1;
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
std::vector<movement::node> movement::find_path(start st, waypoint way){
    int sx = st.x; int sy = st.y; int gx = way.x; int gy = way.y;

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
            // Reconstruct path into a vector
            std::vector<node> path;
            int cur = goal;
            while (cur != -1) {
                int x, y;
                to_coord(cur, x, y);
                node n(x, y);
                path.push_back(n);
                cur = parent[cur];
            }
            // reverse to start → goal
            std::reverse(path.begin(), path.end());
            return path;
        }

        int cx, cy;
        to_coord(current, cx, cy);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (da_grid[ny][nx] == 1) continue;

            int neighbor = to_index(nx, ny);
            if (closed[neighbor]) continue;

            int tentative_g = g_cost[current] + 1;
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
    int sx = st.x; int sy = st.y;

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
            // Reconstruct path into a vector
            std::vector<node> path;
            int cur = goal;
            while (cur != -1) {
                int x, y;
                to_coord(cur, x, y);
                node n(x, y);
                path.push_back(n);
                cur = parent[cur];
            }
            // reverse to start → goal
            std::reverse(path.begin(), path.end());
            return path;
        }

        int cx, cy;
        to_coord(current, cx, cy);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (da_grid[ny][nx] == 1) continue;

            int neighbor = to_index(nx, ny);
            if (closed[neighbor]) continue;

            int tentative_g = g_cost[current] + 1;
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
std::vector<movement::node> movement::find_path(int sx, int sy, waypoint way){
    int gx = way.x; int gy = way.y;

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
            // Reconstruct path into a vector
            std::vector<node> path;
            int cur = goal;
            while (cur != -1) {
                int x, y;
                to_coord(cur, x, y);
                node n(x, y);
                path.push_back(n);
                cur = parent[cur];
            }
            // reverse to start → goal
            std::reverse(path.begin(), path.end());
            return path;
        }

        int cx, cy;
        to_coord(current, cx, cy);

        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int i = 0; i < 4; i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];

            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (da_grid[ny][nx] == 1) continue;

            int neighbor = to_index(nx, ny);
            if (closed[neighbor]) continue;

            int tentative_g = g_cost[current] + 1;
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