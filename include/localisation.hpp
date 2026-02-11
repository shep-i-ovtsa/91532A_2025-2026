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
    double theta_deg = 0.0;
    double process_theta(double theta);
public:
    void set_pose(int new_x, int new_y, double new_theta);


    int get_x() const;
    int get_y() const;
    double get_theta() const;
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
    double heading_deg;
};
class localisation {
private:

    pros::Distance& Left_Sensor;
    pros::Distance& Right_Sensor;
    pros::Distance& Back_Sensor;
    pros::Distance& Front_Sensor;
    pros::Imu& imu;
    bool which_Sensor(double theta_deg);
    void update_position(int x_mm, int y_mm, double theta_deg);
    sensor_data read_sensors();
    void triangulate_position();    
    position current_position;
public:
    void update();
    localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu);
    position get_current_pose() const;
    position get_predicted_pose(double time) const;
     //!passing distance sensor copies are EXPENSIVE, so we pass by reference so we pass references instead
};

#endif