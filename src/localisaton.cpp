#include "localisation.hpp"
#include "timeMaster.hpp"
#include <math.h>
const double FIELD_WIDTH = 2500.0;
const double FIELD_HEIGHT = 2500.0;

const int CONFIDENCE_MINIMUM = 20;
const double JUMP_THRESHOLD = 200.0;
const double SMOOTH = 0.25;

void position::set_pose(int new_x, int new_y, double new_theta) {
    x_mm = new_x;
    y_mm = new_y;
    theta_deg = process_theta(new_theta);

}
double position::process_theta(double theta) {
    theta = fmod(theta, 360.0);
    if (theta < 0)
        theta += 360.0;
    return theta;
}


//----------------------------------------------------------------------------------------
static double deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

void localisation::update_position(int x_mm, int y_mm, double theta_deg) {
    current_position.set_pose(x_mm, y_mm, theta_deg);
}
sensor_data localisation::read_sensors() {
    sensor_data data;
    data.Left_Sensor_mm = Left_Sensor.get();
    data.left_confidence = Left_Sensor.get_confidence();
    data.Right_Sensor_mm = Right_Sensor.get();
    data.right_confidence = Right_Sensor.get_confidence();
    data.Back_Sensor_mm = Back_Sensor.get();
    data.back_confidence = Back_Sensor.get_confidence();
    data.Front_Sensor_mm = Front_Sensor.get();
    data.front_confidence = Front_Sensor.get_confidence();
    data.heading_deg = imu.get_heading();
    return data;
}
void localisation::triangulate_position() {

}
localisation::localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu)
    : Left_Sensor(left_sensor), Right_Sensor(right_sensor), Back_Sensor(back_sensor), Front_Sensor(front_sensor) , imu(imu) {
    update_position(0, 0, 0);
    
}
position localisation::get_current_pose() const{
    return current_position;
}
position localisation::get_predicted_pose(double time) const{
    return current_position; // todo work on prediction logic
}
void localisation::update() {
    sensor_data data = read_sensors();
    //todo finish update logic
}
bool localisation::which_Sensor(double theta_deg) {
    double theta = deg_to_rad(theta_deg);
    return std::abs(std::cos(theta)) >= std::abs(std::sin(theta));
}
//todo add some weighted equations to determine which sensors are more correct xd
