#include "localisation.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "timeMaster.hpp"
#include <cmath>
#include <cstdio>
float trig_table::sin_table[360];
float trig_table::cos_table[360];
const float FIELD_WIDTH  = 3650.0f;
const float FIELD_HEIGHT = 3650.0f;
const float SMOOTH = 0.25f; //REMEMBER the f at the end tells the compiler to make the numebr a float. otherwise it defaults to doubles which take a whole extra cycle for compute
math_params::math_params(){
    this -> bat_volt = pros::battery::get_voltage();
}
void math_params::grab_volt(){
    this -> bat_volt = pros::battery::get_voltage();
}
void math_params::set_smoothing(float smooth){
    this -> smoothing = smooth;
}
void math_params::set_epsilon(float eps){
    this -> epsilon = eps;
}
void math_params::set_max_ray(float max){
    this -> max_ray = max;
}
void math_params::set_sensor_params(int max, int min){
    this -> max_sensor_mm = max;
    this -> min_sensor_mm = min;
}
void math_params::adjust_cred(float front, float back, float left, float right){
    this -> front_cred = front;
    this -> back_cred = back;
    this -> left_cred = left;
    this -> right_cred = right;
}


static float ray_to_wall_distance(
    float px, float py,
    float dx, float dy,
    math_params& params)
{
    float t_min = params.max_ray;

    //!calculate interferance with vertical walls
    if (fabsf(dx) > params.epsilon){ //!floating point plssss!! remmeber to avoid doubles for embedded programming. fabs is floating poitn abs
        float t1 = (0.0f - px) / dx;
        float t2 = (params.max_ray - px) / dx;

        if (t1 > 0 && t1 < t_min) t_min = t1;
        if (t2 > 0 && t2 < t_min) t_min = t2;
    }

    //!calculate interference with horizontal walls
    if (fabsf(dy) > params.epsilon){
        float t3 = (0.0f - py) / dy;
        float t4 = (params.max_ray - py) / dy;

        if (t3 > 0 && t3 < t_min) t_min = t3;
        if (t4 > 0 && t4 < t_min) t_min = t4;
    }

    return t_min;
}
trig_table::trig_table(){
    for(int i = 0; i < 360; i++){
       float r = i * 0.01745329252f; //conversion for radians 
       trig_table::sin_table[i] = sinf(r);
       trig_table::cos_table[i] = cosf(r);
    }
}

float trig_table::sin(int deg){
    deg %= 360;
        if(deg < 0) deg += 360; // ik it looks weird but if someone puts in a weird negative angle or something, this processes it back to a usable value
    return this -> sin_table[deg];
}

float trig_table::cos(int deg){
    deg %= 360;
        if(deg < 0) deg += 360;
    return this -> cos_table[deg];
}

void position::set_pose(float new_x, float new_y, float new_theta){
    x_mm = new_x;
    y_mm = new_y;
    theta_deg = process_theta(new_theta);
}

inline int position::process_theta(int theta){
    theta %= 360;
        if (theta < 0) theta += 360;
    return theta;
}

int position::get_x() const { return x_mm; }
int position::get_y() const { return y_mm; }
int position::get_theta() const { return theta_deg; }

//----------------------------------------------------------------

static trig_table trig; //global sin and cos lookup table for speed >:3

void localisation::update_position(int x_mm, int y_mm, double theta_deg) {
    current_position.set_pose(x_mm, y_mm, (int)theta_deg);
}
void localisation::triangulate_position(sensor_data& data){
    current_position.set_pose(current_position.get_x(),current_position.get_y(),data.heading_deg);
    triangulate_x();
    triangulate_y();
}
void localisation::triangulate_x(){

    int theta = current_position.get_theta();

    float c = trig.cos(theta);
    float s = trig.sin(theta);

    float x_accumalated = 0.0f; 
    float weight_sum = 0.0f;

    float px = (float)current_position.get_x();
    float py = (float)current_position.get_y();

    float voltage_scale = params.bat_volt / params.max_volt; //adjust for voltage
    //front sensor
    {
        float dx =  c;
        float dy =  s;

        float measured = ((float)current_sensor_data.Front_Sensor_mm + current_offset.front) *  voltage_scale;

        if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){

            float t_wall = ray_to_wall_distance(px, py, dx, dy, this -> params);

            if (t_wall < params.max_ray){ //! check if the point is valid

                float x_wall = px + t_wall * dx;

                //! refind position from measurements
                float x_candidate = x_wall - measured * dx;

                float distance_factor = 1.0f - (measured / params.max_ray);
                if (distance_factor < 0.1f) distance_factor = 0.1f;

                float w = fabsf(dx) * distance_factor * params.back_cred; //! horizontal strength
                x_accumalated += x_candidate * w;
                weight_sum += w;
            }
        }
    }

    //============================================================
    //back sensor
    {
        float dx = -c;
        float dy = -s;

        float measured = ((float)current_sensor_data.Back_Sensor_mm + current_offset.back) * voltage_scale;

        if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){

            float t_wall = ray_to_wall_distance(px, py, dx, dy, this -> params);

            if (t_wall < params.max_ray){

                float x_wall = px + t_wall * dx;

                float x_candidate = x_wall - measured * dx;

                float distance_factor = 1.0f - (measured / params.max_ray);
                if (distance_factor < 0.1f) distance_factor = 0.1f;

                float w = fabsf(dx) * distance_factor * params.back_cred;
                x_accumalated += x_candidate * w;
                weight_sum += w;
            }
        }
    }

    //============================================================

    if (weight_sum > 0.0001f){

        float x_estimate = x_accumalated / weight_sum;

        //! smoothing so it dosent go everywhere obs
        float x_filtered = px * (1.0f - params.smoothing) + x_estimate * params.smoothing;

        current_position.set_pose((int)x_filtered, current_position.get_y(), theta);
    }
}

void localisation::triangulate_y(){
    int theta = current_position.get_theta();

    float c = trig.cos(theta);
    float s = trig.sin(theta);

    float y_accum = 0.0f;
    float weight_sum = 0.0f;

    float px = (float)current_position.get_x();
    float py = (float)current_position.get_y();
    float voltage_scale = params.bat_volt / params.max_volt; //so like we return a percentage of the battery from the current volts compared to max voolts
    //so that we can adjust for the amount of trust lost at lower voltages

    // right sensor
    float dx = -s;
    float dy =  c;
    float measured = ((float)current_sensor_data.Right_Sensor_mm + current_offset.right)*voltage_scale;

    if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
        float t_wall = ray_to_wall_distance(px, py, dx, dy, this -> params);
        if (t_wall < params.max_ray){
            float y_wall = py + t_wall * dy;

            float y_candidate = y_wall - measured * dy;
            float distance_factor = 1.0f - (measured / params.max_ray);
            if (distance_factor < 0.1f) distance_factor = 0.1f;

            float w = fabsf(dy) * distance_factor * params.right_cred; //! vertical information strength
            y_accum += y_candidate * w;
            weight_sum += w;
        }
    }

    //left sensor
    
    dx =  s;
    dy = -c;

    measured = ((float)current_sensor_data.Left_Sensor_mm + current_offset.left)*voltage_scale;
    if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
        float t_wall = ray_to_wall_distance(px, py, dx, dy, this -> params);
        if (t_wall < params.max_ray){
            float y_wall = py + t_wall * dy;
            float y_candidate = y_wall - measured * dy;
            float distance_factor = 1.0f - (measured / params.max_ray);
            if (distance_factor < 0.1f) distance_factor = 0.1f;
            float w = fabsf(dy) * distance_factor * params.left_cred;
            y_accum += y_candidate * w;
            weight_sum += w;
        }
    }
    

    //============================================================

    if (weight_sum > 0.0001f){
        float y_est = y_accum / weight_sum;
        float y_filtered = py * (1.0f - params.smoothing) + y_est * params.smoothing;
        current_position.set_pose(current_position.get_x(), y_filtered, theta);
    }
}
void localisation::estimate_obsticles(){
    //todo integrate obsticle interference estimation to math
    //* im too tired for ts rn all of that top math was fucking HORRIBLE xd >.< FAHHHHHH never make me do fuck as trig like that AGAIN */
}

localisation::localisation(pros::Distance& left_sensor,pros::Distance& right_sensor,pros::Distance& back_sensor,pros::Distance& front_sensor,pros::Imu& imu)
    : Left_Sensor(left_sensor),Right_Sensor(right_sensor),Back_Sensor(back_sensor),Front_Sensor(front_sensor),imu(imu){
    imu.set_heading(0);
    set_offset(0, 0, 0, 0);
}

position localisation::get_current_pose() const {
    return current_position;
}

void localisation::update(){
    update_sensors(current_sensor_data);
    params.grab_volt();
    triangulate_position(current_sensor_data);

}

void localisation::set_offset(int back, int front, int left, int right){
    offsets offset;
    offset.back = back;
    offset.front = front;
    offset.left = left;
    offset.right = right;
    current_offset = offset;
}

long localisation::get_horizontal_vision(){
    return (long)(((current_sensor_data.Left_Sensor_mm +current_sensor_data.Right_Sensor_mm) /FIELD_WIDTH) * 100.0);
}

long localisation::get_vertical_vision(){
    return (long)(((current_sensor_data.Back_Sensor_mm +current_sensor_data.Front_Sensor_mm) /FIELD_HEIGHT) * 100.0);
}
void localisation::update_sensors(sensor_data& data){
    data.Left_Sensor_mm = Left_Sensor.get(); 
    data.left_confidence = Left_Sensor.get_confidence();
    data.Right_Sensor_mm = Right_Sensor.get();
    data.right_confidence = Right_Sensor.get_confidence();
    data.Back_Sensor_mm = Back_Sensor.get();
    data.back_confidence = Back_Sensor.get_confidence();
    data.Front_Sensor_mm = Front_Sensor.get();
    data.front_confidence = Front_Sensor.get_confidence();
    data.heading_deg = imu.get_heading();
    data.x_gs = imu.get_accel().x;
    data.y_gs = imu.get_accel().y;
}
void localisation::background_update_process(void* param) {
    localisation* self = static_cast<localisation*>(param);

    while(true) {
        self->update();
        pros::delay(pros::competition::is_autonomous() ? 20 : 500);
    }
}