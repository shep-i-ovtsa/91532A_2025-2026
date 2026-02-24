#ifndef MTP_HPP
#define MTP_HPP
#include "localisation.hpp"
#include <list>
struct obstruction{
public:
    int x;
    int x2;
    int y;
    int y2;
};
struct node{
public:
    int x;
    int y;
    int theta;
    float pos_x;
    float pos_y;
};
struct check_point{
public:
    int left_read;
    int right_read;
    int back_read;
    int front_read;
    float theta;
};
class movement{

public:

    movement();
    static constexpr float FIELD_WIDTH = 3860.0;
    static constexpr float FIELD_HEIGHT = 3860.0;
    static constexpr float res = 50.0; //change this for higher resolution
    //! in mms btw
    void add_obstruction(obstruction obs);    
    int horizontal_nodes = FIELD_WIDTH /res;
    int vertical_nodes = FIELD_HEIGHT /res;
private:
    node node_list[static_cast<int>(FIELD_WIDTH/res)][static_cast<int>(FIELD_WIDTH/res)];
    localisation loco;

};
#endif
/*
//* 1:get from one point to another
//* A:split the field into nodes by resoultion
//        -How do we split the field into resolution?
//        -1:We can take the field dimmensions and divide them by res
//s            take those leftover points and turn them into nodes
//    B:store nodes in a list to be iterated over
//        -just use a list and a set of field coordinates depending on res;
        -remove obscured nodes from the list to avoid including in list

//* C:Drive from node to node
    feed our nodes to an a* algorythim and feed that back into a movement chain
    follow the movement chain between nodes to find the fastest route
*/