#include "localisation.hpp"
#include "pros/rtos.hpp"
#include "timeMaster.hpp"
#include <cmath>
const double FIELD_WIDTH = 3650.0;
const double FIELD_HEIGHT = 3650.0;

const double SMOOTH = 0.25;

void position::set_pose(int new_x, int new_y, int new_theta) {
    x_mm = new_x;
    y_mm = new_y;
    theta_deg = process_theta(new_theta);

}
inline int position::process_theta(int theta) {
    theta %= 360;
    if (theta < 0) theta += 360;
    return theta;
}


//----------------------------------------------------------------------------------------
static inline float deg_to_rad(int deg) {
    return deg * 0.01745329252f; // PI / 180
}

void localisation::update_position(int x_mm, int y_mm, double theta_deg) {
    current_position.set_pose(x_mm, y_mm, theta_deg);
}
void localisation::update_sensors(sensor_data& data) {

    data.Left_Sensor_mm = Left_Sensor.get() + this -> current_offset.left;
    data.left_confidence = Left_Sensor.get_confidence();
    data.Right_Sensor_mm = Right_Sensor.get() + this -> current_offset.right;
    data.right_confidence = Right_Sensor.get_confidence();
    data.Back_Sensor_mm = Back_Sensor.get() + this -> current_offset.back;
    data.back_confidence = Back_Sensor.get_confidence();
    data.Front_Sensor_mm = Front_Sensor.get() + this -> current_offset.front;
    data.front_confidence = Front_Sensor.get_confidence();
    data.heading_deg = imu.get_heading();
}
void localisation::triangulate_position(sensor_data& data) {
    //do some math and 32bit optimised trig
    current_position.set_pose(data.Left_Sensor_mm, data.Back_Sensor_mm, data.heading_deg); //! placeholders
}
localisation::localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu)
    : Left_Sensor(left_sensor), Right_Sensor(right_sensor), Back_Sensor(back_sensor), Front_Sensor(front_sensor) , imu(imu) {
    update_position(0, 0, 0);
    imu.set_heading(0);
    set_offset(0, 0, 0, 0);
}
position localisation::get_current_pose() const{
    return current_position;
}
void localisation::update() {
    update_sensors(current_sensor_data);
    triangulate_position(current_sensor_data);

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
long localisation::get_vertical_vision(){return round(((current_sensor_data.Back_Sensor_mm+current_sensor_data.Front_Sensor_mm)/FIELD_WIDTH)*100);}

void localisation::background_update_process(void* param) {
    localisation* self = static_cast<localisation*>(param);

    const int period = 20;
    int last = pros::millis();

    while (true) {
        int now = pros::millis();

        if (now - last >= period) {
            last += period;
            self->update();
        }

        pros::delay(1);
    }
}

//todo add some weighted equations to determine which sensors are more correct xd
//todo add x coordinate calculation
//todo add y coordinate calculation\
//todo choose a smoothing filter to reduce mm jiter even if within 4mm of jitter at 90%
//todo finish final triangulation function
//todo finish update loop
//todo finish prediciton logic
    //!make sure to use time Master for accuracte timing