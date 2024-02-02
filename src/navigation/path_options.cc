#include "path_options.h"
#include <cstdio>
#include <iostream>
#include <vector>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

float Run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& robot_config) {
    float max_accel = robot_config.max_accel;
    float max_decel = robot_config.max_decel;
    float max_vel = robot_config.max_vel;
    float dt = robot_config.dt;
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


// function to find free path length
// given point cloud (robot frame), curvature
void GetFreePathLength(float curvature, const vector<Eigen::Vector2f>& point_cloud) {

}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> SamplePathOptions(int num_options,
                                                    float max_curvature,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config) {
    vector<navigation::PathOption> * path_options = new vector<navigation::PathOption>(num_options);

    // // loop through curvature from max to -max
    // for (int i = 0; i < num_options; i++) {
    //     float curvature = -1 * max_curvature + (2 * max_curvature) * i / (num_options - 1);

    // }

}

