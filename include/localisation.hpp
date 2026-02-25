#ifndef LOCALISATION_HPP
#define LOCALISATION_HPP

#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include <vector>
#include <atomic>


struct position {
private:
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float theta_deg = 0.0f;
    static int normalize_theta(int theta);
public:
    void set_pose(float x, float y, float theta);
    void set_x(float x){ x_mm = x; }
    void set_y(float y){ y_mm = y; }
    void set_theta(float t){ theta_deg = normalize_theta((int)t); }

    float get_x() const { return x_mm; }
    float get_y() const { return y_mm; }
    float get_theta() const { return theta_deg; }
};


struct sensor_data {
    int Front_Sensor_mm = 0;
    int Back_Sensor_mm = 0;
    int Left_Sensor_mm = 0;
    int Right_Sensor_mm = 0;

    int front_conf = 0;
    int back_conf = 0;
    int left_conf = 0;
    int right_conf = 0;

    int front_speed = 0; //detected speed
    int back_speed = 0;
    int left_speed = 0;
    int right_speed = 0;

    float heading_deg = 0;

    float f_front = 0;
    float f_back  = 0;
    float f_left  = 0;
    float f_right = 0;
};

class localisation {
private:

    pros::Task background_task;

    position predicted_pose;
    position estimated_pose;
    position fused_pose;



    sensor_data real_sensors;

    pros::Distance& left;
    pros::Distance& right;
    pros::Distance& back;
    pros::Distance& front;
    pros::Imu& imu;

    const uint8_t* grid = nullptr;
    int grid_w = 0;
    int grid_h = 0;
    float resolution = 25.0f; // mm per cell

    std::atomic<bool> running{false};

    bool robot_moving = false;
    bool dynamic_environment = false;

    void update_real_sensors();
    float simulate_ray(float x, float y, float theta_deg) const;
    void simulate_sensors_at(float x, float y, float theta,
                             sensor_data& predicted) const;
    float pose_error(const sensor_data& predicted) const;
    void correct_pose_local();

    static void background_task_fn(void* param);

public:
    enum CorrectionState : uint8_t{ 
        CORR_LOW = 0, 
        CORR_HIGH = 1 
    };    
    CorrectionState correction_confidence; 
    localisation(pros::Distance& l,
                 pros::Distance& r,
                 pros::Distance& b,
                 pros::Distance& f,
                 pros::Imu& imu);

    void set_grid(const uint8_t* g, int w, int h, float res);
    void set_pose(float x, float y, float theta);

    void start();
    void stop();
    void update(); // manual update

position& get_pose();

position& get_predicted_pose();

position& get_estimated_pose();
sensor_data& get_real_sensors();
};

#endif