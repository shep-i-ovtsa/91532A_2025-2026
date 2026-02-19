#ifndef LOCALISATION_HPP
#define LOCALISATION_HPP
//-----------------------------------------------------------
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"

struct position {
private:
    float x_mm = 0;
    float y_mm = 0; 
    float theta_deg = 0.0; 
    static int process_theta(int theta);
public:
    void set_pose(float new_x, float new_y, float new_theta);


    int get_x() const;
    int get_y() const;
    int get_theta() const;
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
};
struct math_params{
    float epsilon= 0.0001f;
    void set_epsilon(float eps);
    float max_ray=3650.0f;
    void set_max_ray(float max);
    float smoothing=0.25f;
    void set_smoothing(float smooth);
    float min_sensor_mm = 0;
    float max_sensor_mm = 2500; //v5 sensors are rated for 2.5m before they start returning absurd numbers like 9999
    void set_sensor_params(int max, int min);

    float bat_volt;
    float max_volt = 12700.0f;
    void grab_volt();

    float front_cred;
    float back_cred;
    float left_cred;
    float right_cred;
    void adjust_cred(float front, float back, float left, float right);
    math_params();
};
struct trig_table{
    private:
    static float sin_table[360];
    static float cos_table[360];
    public:
    float sin(int deg);
    float cos(int deg);
    trig_table();
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
    void apply_filter();
    void estimate_obsticles();

public:
    math_params params;
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