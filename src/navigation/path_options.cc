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
float GetFreePathLength(float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {

    float free_path_length = 20; // large number

    // TODO: check that curvature is not 0
    // check if radians or degrees
    Vector2f c = Vector2f(0, 1 / curvature);
    for (int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float r_inner = c[1] - robot_config.width / 2;
        float r_tr = (Vector2f(0, r_inner) - Vector2f(robot_config.length - robot_config.base_link_offset, 0)).norm();

        float theta = atan2(p[0], c[1] - p[1]); // angle between p and c

        if (r_inner <= r_p && r_p <= r_tr) {
            float omega = arccos(r_inner / r_p);
            float length = (theta - omega) * r_p;
        }

        // other cases ...

        free_path_length = min(free_path_length, length);
    }
    return free_path_length;
}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> SamplePathOptions(int num_options,
                                                    float max_curvature,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config) {
    static vector<navigation::PathOption> path_options;
    path_options.clear();

    // loop through curvature from max to -max
    for (int i = 0; i < num_options; i++) {
        float curvature = -1 * max_curvature + (2 * max_curvature) * i / (num_options - 1);
        float free_path_length = GetFreePathLength(curvature, point_cloud);
        navigation::PathOption path_option;
        path_option.curvature = curvature;
        path_option.free_path_length = free_path_length;
        path_options.push_back(path_option);
    }
    return path_options;
}

