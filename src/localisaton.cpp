#include "localisation.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include "timeMaster.hpp"
#include <cstdio>
float trig_table::sin_table[360];
float trig_table::cos_table[360];
const float FIELD_WIDTH  = 3650.0f;
const float FIELD_HEIGHT = 3650.0f;
const float SMOOTH = 0.25f; //REMEMBER the f at the end tells the compiler to make the numebr a float. otherwise it defaults to doubles which take a whole extra cycle for compute





static float ray_to_wall_distance(
    float px, float py,
    float dx, float dy,
    math_params& params)
{
    float t_min = params.max_ray;

    //!calculate interferance with vertical walls
    if (fabsf(dx) > params.epsilon){ //!floating point plssss!! remmeber to avoid doubles for embedded programming. fabs is floating poitn abs
        float t1 = (0.0f - px) / dx;
        float t2 = (FIELD_WIDTH - px) / dx;


        if (t1 > 0 && t1 < t_min) t_min = t1;
        if (t2 > 0 && t2 < t_min) t_min = t2;
    }

    //!calculate interference with horizontal walls
    if (fabsf(dy) > params.epsilon){
        float t3 = (0.0f - py) / dy;
        float t4 = (FIELD_HEIGHT - py) / dy;

        if (t3 > 0 && t3 < t_min) t_min = t3;
        if (t4 > 0 && t4 < t_min) t_min = t4;
    }

    return t_min;
}
static float ray_to_circle_distance(float px, float py,float dx, float dy,const obsticle& obsticle,float max_ray){
    float ox = px - (float)obsticle.x;
    float oy = py - (float)obsticle.y;
    float b = 2.0f * (dx*ox+dy*oy);
    float c = ox*ox + oy*oy - (float)(obsticle.radius*obsticle.radius);

    float discrimination = b*b - 4.0f*c;
    if (discrimination < 0.0f) return max_ray;
    float sqrt_d = sqrtf(discrimination);
    float t1 = (-b - sqrt_d) * 0.5f;
    float t2 = (-b + sqrt_d) * 0.5f;
    float t = max_ray;
    if (t1 > 0.0f && t1 < t) t = t1;
    if (t2 > 0.0f && t2 < t) t = t2;

    return t;
}
float ray_to_nearest_obstacle(float px, float py,float dx, float dy,const std::vector<obsticle>& obstacles,math_params& params){
    float t_min = params.max_ray;
    for (const auto& o : obstacles) {
        float t = ray_to_circle_distance(px, py, dx, dy, o, params.max_ray);
        if (t < t_min) t_min = t;
    }
    return t_min;
}



static trig_table trig; //global sin and cos lookup table for speed >:3

void localisation::triangulate_x() {

    int theta = current_position.get_theta();
    float c = trig.cos(theta);
    float s = trig.sin(theta);

    float px = current_position.get_x();
    float py = current_position.get_y();

    float x_sum = 0.0f;
    float w_sum = 0.0f;

    // FRONT
    if (fabsf(c) > 0.5f) {  // only when mostly horizontal

        float d = current_sensor_data.Front_Sensor_mm;

        if (d > params.min_sensor_mm && d < params.max_sensor_mm) {

            float x_candidate;

            if (c > 0)  // facing east
                x_candidate = FIELD_WIDTH - d;
            else        // facing west
                x_candidate = d;

            float w = fabsf(c) *
                      (current_sensor_data.front_confidence / 100.0f) *
                      params.front_cred;

            x_sum += x_candidate * w;
            w_sum += w;
        }
    }

    // BACK
    if (fabsf(c) > 0.5f) {

        float d = current_sensor_data.Back_Sensor_mm;

        if (d > params.min_sensor_mm && d < params.max_sensor_mm) {

            float x_candidate;

            if (c > 0)
                x_candidate = d;
            else
                x_candidate = FIELD_WIDTH - d;

            float w = fabsf(c) *
                      (current_sensor_data.back_confidence / 100.0f) *
                      params.back_cred;

            x_sum += x_candidate * w;
            w_sum += w;
        }
    }

    if (w_sum > 0.01f) {
        float x_est = x_sum / w_sum;
            
        float x_prev = current_position.get_x();

        float alpha = params.smoothing;

        // optional: adaptive smoothing from acceleration
        float accel_mag = sqrtf(
            current_sensor_data.x_gs * current_sensor_data.x_gs +
            current_sensor_data.y_gs * current_sensor_data.y_gs
        );

        float accel_factor = fminf(accel_mag / params.accel_max, 1.0f);
        alpha += accel_factor * params.accel_gain;
        alpha = fminf(alpha, params.smoothing_max);

        // EMA
        float x_filtered = alpha * x_est + (1.0f - alpha) * x_prev;

        current_position.set_x(x_filtered);
    }
}

void localisation::triangulate_y() {

    int theta = current_position.get_theta();
    float c = trig.cos(theta);
    float s = trig.sin(theta);

    float px = current_position.get_x();
    float py = current_position.get_y();

    float y_sum = 0.0f;
    float w_sum = 0.0f;

    // RIGHT
    if (fabsf(s) > 0.5f) {

        float d = current_sensor_data.Right_Sensor_mm;

        if (d > params.min_sensor_mm && d < params.max_sensor_mm) {

            float y_candidate;

            if (s > 0)   // facing north
                y_candidate = d;
            else
                y_candidate = FIELD_HEIGHT - d;

            float w = fabsf(s) *
                      (current_sensor_data.right_confidence / 100.0f) *
                      params.right_cred;

            y_sum += y_candidate * w;
            w_sum += w;
        }
    }

    // LEFT
    if (fabsf(s) > 0.5f) {

        float d = current_sensor_data.Left_Sensor_mm;

        if (d > params.min_sensor_mm && d < params.max_sensor_mm) {

            float y_candidate;

            if (s > 0)
                y_candidate = FIELD_HEIGHT - d;
            else
                y_candidate = d;

            float w = fabsf(s) *
                      (current_sensor_data.left_confidence / 100.0f) *
                      params.left_cred;

            y_sum += y_candidate * w;
            w_sum += w;
        }
    }

    if (w_sum > 0.01f) {
        float y_est = y_sum / w_sum;
        float y_prev = current_position.get_y();

        float alpha = params.smoothing;

        float accel_mag = sqrtf(
            current_sensor_data.x_gs * current_sensor_data.x_gs +
            current_sensor_data.y_gs * current_sensor_data.y_gs
        );

        float accel_factor = fminf(accel_mag / params.accel_max, 1.0f);
        alpha += accel_factor * params.accel_gain;
        alpha = fminf(alpha, params.smoothing_max);

        float y_filtered = alpha * y_est + (1.0f - alpha) * y_prev;

        current_position.set_y(y_filtered);    
    }
} //stole some math from theese guys :p and fixed our feedback loop ;-;
    //turns out becoming a madwoman at 4am about some geometry is NOT that great of an idea
//credit to https://github.com/jiazegao/RCL-Tracking/tree/main

localisation::localisation(pros::Distance& left_sensor,pros::Distance& right_sensor,pros::Distance& back_sensor,pros::Distance& front_sensor,pros::Imu& imu)
    : Left_Sensor(left_sensor),Right_Sensor(right_sensor),Back_Sensor(back_sensor),Front_Sensor(front_sensor),imu(imu), background_task(background_update_process , this){
    imu.set_heading(0);

    while(imu.is_calibrating()) pros::delay(10);
    set_offset(0, 0, 0, 0);
}
void localisation::set_start_position(int x, int y, int theta){
    update_position(x,y,theta);
}
position& localisation::get_current_pose() {
    return current_position;
}

void localisation::update(){
    triangulate_position();

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
    data.Left_Sensor_mm = Left_Sensor.get() + current_offset.left; 
    data.left_confidence = Left_Sensor.get_confidence();
    data.Right_Sensor_mm = Right_Sensor.get() + current_offset.right;
    data.right_confidence = Right_Sensor.get_confidence();
    data.Back_Sensor_mm = Back_Sensor.get() + current_offset.back;
    data.back_confidence = Back_Sensor.get_confidence();
    data.Front_Sensor_mm = Front_Sensor.get() + current_offset.front;
    data.front_confidence = Front_Sensor.get_confidence();
    data.heading_deg = imu.get_heading();
    data.x_gs = imu.get_accel().x;
    data.y_gs = imu.get_accel().y;
}
volatile bool lazy_switch = false;
void start(){
    lazy_switch = !lazy_switch;
}
void localisation::background_update_process(void* param) {
    localisation* self = static_cast<localisation*>(param);

    while(true) {
        if(lazy_switch){
            self->update();
            pros::delay(pros::competition::is_autonomous() ? 40 : 500);
        } else {
            pros::delay(100);
        }
    }
}