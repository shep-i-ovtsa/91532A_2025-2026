#include "mtp.hpp"
#include <list>
#include "localisation.hpp"
#include "subsystems.hpp"
movement::movement() : loco(left_sensor, right_sensor, back_sensor, front_sensor, imu){
    for(int i = 0; i < vertical_nodes; i++){
        for(int j = 0; j < horizontal_nodes; j++){
            node temp_node;
            temp_node.x = i;
            temp_node.y = j;
            temp_node.theta = 0;
            temp_node.pos_x = i+res;
            temp_node.pos_y = j+res;
            node_list[i][j] = temp_node;
        }
    }
}
void movement::add_obstruction(obstruction obs){
    for(int i = 0; i < obs.y2-obs.y; i++){
    }
}