#include "navigation/path_options.h"
// #include "amrl_msgs/AckermannCurvatureDriveMsg.h"

// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

float 1DTimeOptimalControl() {
    float current_speed = robot_vel_.norm();
    float max_accel = robot_config_->max_accel;
    float max_decel = robot_config_->max_decel;
    float cmd_vel = current_speed;
    float stopping_dist = (current_speed * current_speed) / (2 * max_decel);
    // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
    if (dist_to_go > stopping_dist && current_speed < max_vel) {
        cmd_vel += max_accel * dt;
    }
    else if (dist_to_go > stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
                                                                        // probably needs hysteresis
        cmd_vel = max_vel;
    }
    else {  // otherwise needs to decelerate
        cmd_vel -= max_decel * dt;
    }
    return cmd_vel;
}