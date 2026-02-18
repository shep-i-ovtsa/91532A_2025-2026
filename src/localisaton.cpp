#include "localisation.hpp"
#include "timeMaster.hpp"
#include <atomic>
#include <cmath>
const double FIELD_WIDTH = 3650.0;
const double FIELD_HEIGHT = 3650.0;

const double SMOOTH = 0.25;

void position::set_pose(int new_x, int new_y, double new_theta) {
    x_mm = new_x;
    y_mm = new_y;
    theta_deg = process_theta(new_theta);

}
long position::process_theta(long theta) {
    theta = fmod(theta, 360.0);
    if (theta < 0)
        theta += 360.0;
    return theta;
}


//----------------------------------------------------------------------------------------
static long deg_to_rad(double deg) {
    return deg * M_PI / 180.0;
}

void localisation::update_position(int x_mm, int y_mm, double theta_deg) {
    current_position.set_pose(x_mm, y_mm, theta_deg);
}
sensor_data localisation::read_sensors() {
    sensor_data data;
    data.Left_Sensor_mm = Left_Sensor.get() + current_offset.left;
    data.left_confidence = Left_Sensor.get_confidence();
    data.Right_Sensor_mm = Right_Sensor.get() + current_offset.right;
    data.right_confidence = Right_Sensor.get_confidence();
    data.Back_Sensor_mm = Back_Sensor.get() + current_offset.back;
    data.back_confidence = Back_Sensor.get_confidence();
    data.Front_Sensor_mm = Front_Sensor.get() + current_offset.front;
    data.front_confidence = Front_Sensor.get_confidence();
    data.heading_deg = imu.get_heading();
    return data;
}
void localisation::triangulate_position() {

}
localisation::localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu)
    : Left_Sensor(left_sensor), Right_Sensor(right_sensor), Back_Sensor(back_sensor), Front_Sensor(front_sensor) , imu(imu) {
    update_position(0, 0, 0);
    imu.set_heading(0);
}
position localisation::get_current_pose() const{
    return current_position;
}
void localisation::update() {
    sensor_data data = read_sensors();
    //todo finish update logic
}
bool localisation::which_Sensor(double theta_deg) {
    double theta = deg_to_rad(theta_deg);
    return std::abs(std::cos(theta)) >= std::abs(std::sin(theta));
}
void localisation::set_offset(int back, int front, int left, int right){
    offsets offset;
    offset.back = back;
    offset.front = front;
    offset.left = left;
    offset.right = right;
    this -> current_offset = offset;
}
long localisation::get_horizontal_vision(){return round(((current_sensor_data.Left_Sensor_mm+current_sensor_data.Right_Sensor_mm)/FIELD_WIDTH)*100);}
long localisation::getvertical_vision(){return round(((current_sensor_data.Back_Sensor_mm+current_sensor_data.Front_Sensor_mm)/FIELD_WIDTH)*100);}
//todo add some weighted equations to determine which sensors are more correct xd
//todo add x coordinate calculation
//todo add y coordinate calculation\
//todo choose a smoothing filter to reduce mm jiter
//todo finish final triangulation function
//todo finish update loop
//todo finish prediciton logic
    //!make sure to use time Master for accuracte timing