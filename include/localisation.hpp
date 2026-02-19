#ifndef LOCALISATION_HPP
#define LOCALISATION_HPP
//-----------------------------------------------------------
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"

struct position {
private:
    int x_mm = 0;
    int y_mm = 0;
    int theta_deg = 0.0;
    static int process_theta(int theta);
public:
    void set_pose(int new_x, int new_y, int new_theta);


    int get_x() const;
    int get_y() const;
    int get_theta() const;
};
struct sensor_data {
    int Left_Sensor_mm;
    int left_confidence;
    int Right_Sensor_mm;
    int right_confidence;
    int Back_Sensor_mm;
    int back_confidence;
    int Front_Sensor_mm;
    int front_confidence;
    int heading_deg;
};
struct offsets {
    int back;
    int front;
    int left;
    int right; //!the cortex a9 is a 32 bit processor, optimise for 32 bit / 16 bit operations when possible!
};
class localisation {
private:

    position current_position;
    offsets current_offset;
    sensor_data current_sensor_data;   

    pros::Distance& Left_Sensor;
    pros::Distance& Right_Sensor;
    pros::Distance& Back_Sensor;
    pros::Distance& Front_Sensor;
    pros::Imu& imu;

    void update_position(int x_mm, int y_mm, double theta_deg);
  
    bool which_Sensor(double theta_deg);
    void update_sensors(sensor_data& data);

    void triangulate_position(sensor_data& data);   
    void triangulate_x();
    void triangulate_y(); 

public:
    static void background_update_process(void* param);
    long get_horizontal_vision();
    long get_vertical_vision();
    void update();
    localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu);
    position get_current_pose() const;
    position get_predicted_pose(double time) const;
    void set_offset(int back, int front, int left, int right);
     //!passing distance sensor copies are EXPENSIVE, so we pass by reference so we pass references instead
};

#endif