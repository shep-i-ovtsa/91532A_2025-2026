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

void math_params::set_outlier_threshold(float threshold){
    this -> outlier_threshold_mm = threshold;
}
void math_params::set_blend(float blend){
    this -> obstacle_blend = blend;
}
void math_params::set_accel(float gain, float accel_max, float smoothing_max){
    this -> accel_gain = gain;
    this -> accel_max = accel_max;
    this -> smoothing_max = smoothing_max;
}


void localisation::add_obsticle(obsticle& obs){
    this -> known_obstacles.emplace_back(obs);
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
float localisation::motion_blur() {
    float ax = current_sensor_data.x_gs;
    float ay = current_sensor_data.y_gs;

    float accel_mag = sqrtf(ax * ax + ay * ay);

    float normalized = accel_mag / params.accel_max;
    if (normalized > 1.0f) normalized = 1.0f;

    float alpha =
        params.smoothing +
        params.accel_gain * normalized;

    if (alpha > params.smoothing_max)
        alpha = params.smoothing_max;

    return alpha;
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

    //front sensor
    {
        
        float dx =  c;
        float dy =  s;

        float measured = ((float)current_sensor_data.Front_Sensor_mm); //lowk the voltage dosent mess us up that much so im going to remove that
            if (measured >= params.min_sensor_mm &&measured <= params.max_sensor_mm){
            if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
        float t_wall = ray_to_wall_distance(px, py, dx, dy, this ->params);           
            if (t_wall < params.max_ray){
        float t_obs = params.max_ray;
            if(!known_obstacles.empty())t_obs  = ray_to_nearest_obstacle(px, py, dx, dy, known_obstacles, this -> params);
        float predicted = (t_obs < t_wall) ? t_obs : t_wall;
        float leftover = measured - predicted;
        bool irregular = fabsf(leftover) > params.outlier_threshold_mm;
  //! check if the point is valid
        float effective_measured = measured;
            if (t_obs < t_wall) effective_measured = t_wall;        
            else if (irregular) effective_measured = predicted; 
            else effective_measured = measured;      
        float x_wall = px + t_wall * dx;
        float x_candidate = x_wall - effective_measured * dx;
            float distance_factor = 1.0f - (effective_measured / params.max_ray);
            if (distance_factor < 0.1f) distance_factor = 0.1f;
            float w = fabsf(dx) * distance_factor * params.front_cred;
            w *= (current_sensor_data.front_confidence / 100.0f);
            if (t_obs < t_wall) w *= params.obstacle_blend;
            else if (irregular) w *= 0.1f;
            x_accumalated += x_candidate * w; //*oh god this a big ass block of math >.<
            //*do i feel bad for the person who has to debug this ;-;
            weight_sum += w;
        }
        }
    }}

    //============================================================
    //back sensor
    {
        float dx = -c;
        float dy = -s;

        float measured = ((float)current_sensor_data.Back_Sensor_mm);
        if (measured >= params.min_sensor_mm &&measured <= params.max_sensor_mm){
            if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
            float t_wall = ray_to_wall_distance(px, py, dx, dy, this ->params);
        if (t_wall < params.max_ray){
        float t_obs = params.max_ray;
        if(!known_obstacles.empty())t_obs  = ray_to_nearest_obstacle(px, py, dx, dy, known_obstacles, this -> params);

        float predicted = (t_obs < t_wall) ? t_obs : t_wall;
        float leftover = measured - predicted;
        bool irregular = fabsf(leftover) > params.outlier_threshold_mm;
        float effective_measured = measured;                  
        if (t_obs < t_wall) effective_measured = t_wall;        
        else if (irregular) effective_measured = predicted; 
        else effective_measured = measured;
            float x_wall = px + t_wall * dx;
            float x_candidate = x_wall - effective_measured * dx;
            float distance_factor = 1.0f - (effective_measured / params.max_ray);
        if (distance_factor < 0.1f) distance_factor = 0.1f;
        float w = fabsf(dx) * distance_factor * params.back_cred;
        w *= (current_sensor_data.back_confidence / 100.0f);
        if (t_obs < t_wall) w *= params.obstacle_blend;
        else if (irregular) w *= 0.1f;
        x_accumalated += x_candidate * w;
        weight_sum += w;
        }
        }
    }}

    //============================================================

    if (weight_sum > 0.0001f){

        float x_est = x_accumalated / weight_sum;

        //! smoothing so it dosent go everywhere again
        float alpha = motion_blur();
        float x_filtered = px * (1.0f - alpha) + x_est * alpha;

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

    // right sensor
    {
        
        float dx = -s;
        float dy =  c;

        float measured = ((float)current_sensor_data.Right_Sensor_mm);
            if (measured >= params.min_sensor_mm &&measured <= params.max_sensor_mm){
            if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
        float t_wall = ray_to_wall_distance(px, py, dx, dy, this ->params);           
            if (t_wall < params.max_ray){
        float t_obs = params.max_ray;
            if(!known_obstacles.empty())t_obs  = ray_to_nearest_obstacle(px, py, dx, dy, known_obstacles, this -> params);
        float predicted = (t_obs < t_wall) ? t_obs : t_wall;
        float leftover = measured - predicted;
        bool irregular = fabsf(leftover) > params.outlier_threshold_mm;
        float effective_measured = measured;
            if (t_obs < t_wall) effective_measured = t_wall;        
            else if (irregular) effective_measured = predicted; 
            else effective_measured = measured;
        float y_wall = py + t_wall * dy;
        float y_candidate = y_wall - effective_measured * dy;
            float distance_factor = 1.0f - (effective_measured / params.max_ray);
            if (distance_factor < 0.1f) distance_factor = 0.1f;

            float w = fabsf(dy) * distance_factor * params.right_cred; //! vertical strength
            w *= (current_sensor_data.right_confidence / 100.0f);
            if (t_obs < t_wall) w *= params.obstacle_blend;
            else if (irregular) w *= 0.1f;
            y_accum += y_candidate * w;
            weight_sum += w;
        }
        }
    }}

    //left sensor
    {
        
        float dx =  s;
        float dy = -c;

        float measured = ((float)current_sensor_data.Left_Sensor_mm);
            if (measured >= params.min_sensor_mm &&measured <= params.max_sensor_mm){
            if (fabsf(dx) > params.epsilon || fabsf(dy) > params.epsilon){
        float t_wall = ray_to_wall_distance(px, py, dx, dy, this ->params);           
            if (t_wall < params.max_ray){
        float t_obs = params.max_ray;
            if(!known_obstacles.empty())t_obs  = ray_to_nearest_obstacle(px, py, dx, dy, known_obstacles, this -> params);
        float predicted = (t_obs < t_wall) ? t_obs : t_wall;
        float leftover = measured - predicted;
        bool irregular = fabsf(leftover) > params.outlier_threshold_mm;
        float effective_measured = measured;
            if (t_obs < t_wall) effective_measured = t_wall;        
            else if (irregular) effective_measured = predicted; 
            else effective_measured = measured;
        float y_wall = py + t_wall * dy;
        float y_candidate = y_wall - effective_measured * dy;
            float distance_factor = 1.0f - (effective_measured / params.max_ray);
            if (distance_factor < 0.1f) distance_factor = 0.1f;
            float w = fabsf(dy) * distance_factor * params.left_cred;
            w *= (current_sensor_data.left_confidence / 100.0f);
            if (t_obs < t_wall) w *= params.obstacle_blend;
            else if (irregular) w *= 0.1f;
            y_accum += y_candidate * w;
            weight_sum += w;
        }
        }
    }}

    //============================================================

    if (weight_sum > 0.0001f){

        float y_est = y_accum / weight_sum;

        //! smoothing so it dosent go everywhere obs
        float alpha = motion_blur();
        float y_filtered = py * (1.0f - alpha) + y_est * alpha;

        current_position.set_pose(current_position.get_x(), y_filtered, theta);
    }
}
void estimate_obsticles(){
    //todo integrate obsticle interference estimation to math
    //* im too tired for ts rn all of that top math was fucking HORRIBLE xd >.< FAHHHHHH never make me do fuck as math like this AGAIN */
}

localisation::localisation(pros::Distance& left_sensor,pros::Distance& right_sensor,pros::Distance& back_sensor,pros::Distance& front_sensor,pros::Imu& imu)
    : Left_Sensor(left_sensor),Right_Sensor(right_sensor),Back_Sensor(back_sensor),Front_Sensor(front_sensor),imu(imu){
    imu.set_heading(0);
    set_offset(0, 0, 0, 0);
    pros::Task background_update(background_update_process, this);
}

position& localisation::get_current_pose() {
    return current_position;
}

void localisation::update(){
    update_sensors(current_sensor_data);
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
void localisation::background_update_process(void* param) {
    localisation* self = static_cast<localisation*>(param);

    while(true) {
        self->update();
        pros::delay(pros::competition::is_autonomous() ? 40 : 500);
    }
}