#ifndef LOCALISATION_HPP
#define LOCALISATION_HPP
//-----------------------------------------------------------
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include <cmath>
struct position {
private:
    float x_mm = 0;
    float y_mm = 0; 
    float theta_deg = 0.0; 
    inline float process_theta(float theta){
        theta = fmodf(theta, 360.0f);
        if (theta < 0) theta += 360.0f;
        return theta;
    }
public:

    void set_pose(float new_x, float new_y, float new_theta){
        x_mm = new_x;
        y_mm = new_y;
        theta_deg = process_theta(new_theta);
    }

    float get_x() const{
        return this -> x_mm;
    }
    float get_y() const{
        return this -> y_mm;
    }
    float get_theta() const{
        return this -> theta_deg;
    }
    void set_x(float x){
        this -> x_mm = x;
    }
    void set_y(float y){
        this -> y_mm = y;
    }
    void set_theta(float t){
        this -> theta_deg = t;
    }
};
struct sensor_data {
    int Left_Sensor_mm; //sensors return mms in integers
    int left_confidence;
    int Right_Sensor_mm;
    int right_confidence;
    int Back_Sensor_mm;
    int back_confidence;
    int Front_Sensor_mm;
    int front_confidence;
    float heading_deg;
    float x_gs;
    float y_gs;
    float acceleration;
};
struct offsets {
    int back;
    int front;
    int left;
    int right; //!the cortex a9 is a 32 bit processor, optimise for 32 bit / 16 bit operations when possible!
};
struct obsticle { //! we represent obsticles as a rectangle in our calculations so we can account for interference with our sensors
int x;
int y;
int radius; //todo add obstacle soft detection
std::string name;
};
struct math_params{
    float epsilon= 0.0001f;

    float max_ray=5661.0f; //maximum possible field distance

    float smoothing=0.25f; //smooting for position updates

    float min_sensor_mm = 50; //mimum sensor range to read
    float max_sensor_mm = 2500; //v5 sensors are rated for 2.5m before they start returning absurd numbers like 9999
    void set_smoothing(float smooth){
        this -> smoothing = smooth;
    }
    void set_epsilon(float eps){
        this -> epsilon = eps;
    }
    void set_max_ray(float max){
        this -> max_ray = max;
    }
    void set_sensor_params(int max, int min){
        this -> max_sensor_mm = max;
        this -> min_sensor_mm = min;
    }
    float front_cred = 0.98; //how accurate is the sensor?
    float back_cred = 0.96;
    float left_cred = 0.98;
    float right_cred = 0.97;
    void adjust_cred(float front, float back, float left, float right){
        this -> front_cred = front;
        this -> back_cred = back;
        this -> left_cred = left;
        this -> right_cred = right;
    }

    //maximum jump allowed before labeled as an outlier
    float outlier_threshold_mm= 200.0f;
    float obstacle_blend = 0.7f; //how much do we trust the know obstacle? 0.0 = fully trust - 1.0 NO TRUST ATALL no blending
    void set_outlier_threshold(float threshold){
        this -> outlier_threshold_mm = threshold;
    }    
    void set_blend(float blend){
        this -> obstacle_blend = blend;
    }
    float accel_gain = 0.4f; //how much acceleration affects smoothing
    float accel_max = 1.5f; //max gs under acceleration
    float smoothing_max = 0.7f; //smoothing cap
    void set_accel(float gain, float accel_max, float smoothing_max){
        this -> accel_gain = gain;
        this -> accel_max = accel_max;
        this -> smoothing_max = smoothing_max;
    }    
    math_params();
};
struct trig_table{
    private:
    static float sin_table[360];
    static float cos_table[360];
    public:
    float sin(int deg){
        deg %= 360;
            if(deg < 0) deg += 360; // ik it looks weird but if someone puts in a weird negative angle or something, this processes it back to a usable value
        return this -> sin_table[deg];
    }
    float cos(int deg){
        deg %= 360;
            if(deg < 0) deg += 360;
        return this -> cos_table[deg];
    }
    trig_table(){
    for(int i = 0; i < 360; i++){
       float r = i * 0.01745329252f; //conversion for radians 
       trig_table::sin_table[i] = sinf(r);
       trig_table::cos_table[i] = cosf(r);
    }
}};
class localisation {
private:
    pros::Task background_task;
    position current_position;
    offsets current_offset;
    sensor_data current_sensor_data;   

    pros::Distance& Left_Sensor;
    pros::Distance& Right_Sensor;
    pros::Distance& Back_Sensor;
    pros::Distance& Front_Sensor;
    pros::Imu& imu;

    void update_position(float x_mm, float y_mm, double theta_deg) {
        current_position.set_pose(x_mm, y_mm, theta_deg);
    }  
    bool which_Sensor(double theta_deg);
    void update_sensors(sensor_data& data);
    
    void triangulate_position(){
        update_sensors(current_sensor_data);
        current_position.set_pose(current_position.get_x(),current_position.get_y(),current_sensor_data.heading_deg);
        triangulate_x();
        triangulate_y();
    }  
    
    void triangulate_x();
    void triangulate_y(); 
    float motion_blur();
public:    
    void start();
    math_params params;


    void set_start_position(int x_mm, int y_mm, int theta_deg);
    std::vector<obsticle> known_obstacles; //hold onto know obstacles
    std::vector<obsticle>& get_obsticles(){
        return known_obstacles;
    }
    void add_obsticle(obsticle& obsticle){
        this -> known_obstacles.emplace_back(obsticle);
    }
    void pop_obsticle(){
        this -> known_obstacles.pop_back();
    }    
    

    static void background_update_process(void* param);
    long get_horizontal_vision();
    long get_vertical_vision();
    void update();
    localisation(pros::Distance& left_sensor, pros::Distance& right_sensor, pros::Distance& back_sensor, pros::Distance& front_sensor, pros::Imu& imu);
    position& get_current_pose();
    position get_predicted_pose(double time) const;
    void set_offset(int back, int front, int left, int right);
     //!passing distance sensor copies are EXPENSIVE, so we pass by reference so we pass references instead
};
#endif 