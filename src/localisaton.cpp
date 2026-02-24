#include "localisation.hpp"
#include "pros/rtos.hpp"
#include <cmath>

static constexpr float MAX_RAY_MM = 5000.0f; //max ray we shoot
static constexpr float SENSOR_MAX = 2500.0f; //the range of our sensors
static constexpr float SENSOR_MIN = 50.0f; //min ray

static constexpr float SENSOR_ALPHA = 0.3f;//smooting filter
static constexpr int   CONF_THRESHOLD = 60; //ignore readings below this threashold
static constexpr float CORRECTION_ACCEPT = 4000.0f; //max error we allow before being like "ok thats enough"
static constexpr float DYNAMIC_THRESHOLD = 200.0f; //motion blur

static constexpr float MOVE_THRESHOLD = 100.0f;
static constexpr float VEL_WEIGHT = 0.05f;

int position::normalize_theta(int theta){
    theta %= 360;
    if(theta < 0) theta += 360;
    return theta;
}

void position::set_pose(float x, float y, float theta){
    x_mm = x;
    y_mm = y;
    theta_deg = normalize_theta((int)theta);
}

localisation::localisation(
    pros::Distance& l,
    pros::Distance& r,
    pros::Distance& b,
    pros::Distance& f,
    pros::Imu& imu_)
    : left(l), right(r), back(b), front(f), imu(imu_),
      background_task(background_task_fn, this)
{
    imu.set_heading(0);
    while(imu.is_calibrating()) pros::delay(10);
}

void localisation::set_grid(const uint8_t* g, int w, int h, float res){
    grid = g;
    grid_w = w;
    grid_h = h;
    resolution = res;
}

auto clamp(float v){
    if (v < SENSOR_MIN) return SENSOR_MIN;
    if(v > SENSOR_MAX) return SENSOR_MAX;
    return v;
}

void localisation::update_real_sensors(){

    int raw_front = clamp(front.get());
    int raw_back  = clamp(back.get());
    int raw_left  = clamp(left.get());
    int raw_right = clamp(right.get());

    real_sensors.front_conf = front.get_confidence();
    real_sensors.back_conf  = back.get_confidence();
    real_sensors.left_conf  = left.get_confidence();
    real_sensors.right_conf = right.get_confidence();

    real_sensors.front_speed = front.get_object_velocity();
    real_sensors.back_speed  = back.get_object_velocity();
    real_sensors.left_speed  = left.get_object_velocity();
    real_sensors.right_speed = right.get_object_velocity();

    robot_moving =
        std::fabs(real_sensors.front_speed) > MOVE_THRESHOLD ||
        std::fabs(real_sensors.back_speed)  > MOVE_THRESHOLD;

    dynamic_environment =
        std::fabs(real_sensors.front_speed) > DYNAMIC_THRESHOLD ||
        std::fabs(real_sensors.back_speed)  > DYNAMIC_THRESHOLD ||
        std::fabs(real_sensors.left_speed)  > DYNAMIC_THRESHOLD ||
        std::fabs(real_sensors.right_speed) > DYNAMIC_THRESHOLD;

    float dynamic_alpha = robot_moving ? 0.5f : SENSOR_ALPHA;

    real_sensors.f_front =
        dynamic_alpha * raw_front +
        (1.0f - dynamic_alpha) * real_sensors.f_front;

    real_sensors.f_back =
        dynamic_alpha * raw_back +
        (1.0f - dynamic_alpha) * real_sensors.f_back;

    real_sensors.f_left =
        dynamic_alpha * raw_left +
        (1.0f - dynamic_alpha) * real_sensors.f_left;

    real_sensors.f_right =
        dynamic_alpha * raw_right +
        (1.0f - dynamic_alpha) * real_sensors.f_right;

    real_sensors.heading_deg = imu.get_heading();
}

float localisation::simulate_ray(
    float x, float y, float theta_deg) const
{
    if(!grid) return 0.0f;

    float rad = theta_deg * (M_PI / 180.0f);
    float dx = cosf(rad);
    float dy = sinf(rad);

    float step = resolution * 0.25f;
    float dist = 0.0f;

    while(dist < MAX_RAY_MM){

        float tx = x + dx * dist;
        float ty = y + dy * dist;

        int gx = (int)(tx / resolution);
        int gy = (int)(ty / resolution);

        if(gx < 0 || gy < 0 || gx >= grid_w || gy >= grid_h)
            return dist;

        if(grid[gy * grid_w + gx] == 1)
            return dist;

        dist += step;
    }

    return MAX_RAY_MM;
}

void localisation::simulate_sensors_at(
    float x, float y, float theta,
    sensor_data& predicted) const
{
    predicted.Front_Sensor_mm =
        (int)simulate_ray(x,y,theta);

    predicted.Back_Sensor_mm =
        (int)simulate_ray(x,y,theta+180);

    predicted.Left_Sensor_mm =
        (int)simulate_ray(x,y,theta+90);

    predicted.Right_Sensor_mm =
        (int)simulate_ray(x,y,theta-90);
}

float localisation::pose_error(const sensor_data& predicted) const{

    if(dynamic_environment) return 1e12f;

    float e = 0.0f;

    auto weighted_sq = [](float pred, float real, int conf){
        if(conf < CONF_THRESHOLD) return 0.0f;
        float w = conf / 100.0f;
        float d = pred - real;
        return w * d * d;
    };

    e += weighted_sq(predicted.Front_Sensor_mm,real_sensors.f_front,real_sensors.front_conf);
    e += weighted_sq(predicted.Back_Sensor_mm,real_sensors.f_back,real_sensors.back_conf);
    e += weighted_sq(predicted.Left_Sensor_mm,real_sensors.f_left, real_sensors.left_conf);
    e += weighted_sq(predicted.Right_Sensor_mm,real_sensors.f_right,real_sensors.right_conf);

    float vel_penalty =
        std::fabs(real_sensors.front_speed) +
        std::fabs(real_sensors.back_speed);

    e += VEL_WEIGHT * vel_penalty;

    return e;
}

void localisation::correct_pose_local(){

    if(robot_moving || dynamic_environment) return;

    float base_x = current_pose.get_x();
    float base_y = current_pose.get_y();
    float base_t = real_sensors.heading_deg;

    float best_x = base_x;
    float best_y = base_y;
    float best_t = base_t;

    float best_error = 1e12f;
    sensor_data predicted;

    const int pos_steps[3] = {50, 20, 5};
    const int ang_steps[2] = {3, 1};

    for(int stage = 0; stage < 3; ++stage){

        int pos_step = pos_steps[stage];

        for(int dx = -pos_step; dx <= pos_step; dx += pos_step){
            for(int dy = -pos_step; dy <= pos_step; dy += pos_step){

                for(int a = 0; a < 2; ++a){
                    int ang_step = ang_steps[a];

                    for(int dt = -ang_step; dt <= ang_step; dt += ang_step){

                        float test_x = best_x + dx;
                        float test_y = best_y + dy;
                        float test_t = best_t + dt;

                        simulate_sensors_at(test_x, test_y,test_t, predicted);
                        float err = pose_error(predicted);

                        if(err < best_error){
                            best_error = err;
                            best_x = test_x;
                            best_y = test_y;
                            best_t = test_t;
                        }
                    }
                }
            }
        }
    }

    if(best_error < CORRECTION_ACCEPT){
        current_pose.set_pose(best_x, best_y, best_t);
    }
}

void localisation::update(){
    update_real_sensors();
    correct_pose_local();
}

void localisation::background_task_fn(void* param){
    localisation* self = static_cast<localisation*>(param);

    while(true){
        if(self->running.load()){
            self->update();
            pros::delay(40);
        } else {
            pros::delay(100);
        }
    }
}

void localisation::start(){
    running.store(true);
}

void localisation::stop(){
    running.store(false);
}

position& localisation::get_pose(){
    return current_pose;
}

sensor_data& localisation::get_real_sensors(){
    return real_sensors;
}

void localisation::set_pose(float x, float y, float theta){
    current_pose.set_pose(x,y,theta);
}