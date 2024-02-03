#include "path_options.h"
#include <cstdio>
#include <iostream>
// #include <cmath>
// 1d time optimal control
// given distance to go, max decel, max vel
// out vel

// do this on path option after selecting it

float run1DTimeOptimalControl(float dist_to_go, float current_speed, const navigation::NavigationParams& robot_config) {
    float max_accel = robot_config.max_accel;
    float max_decel = robot_config.max_decel;
    float max_vel = robot_config.max_vel;
    float dt = robot_config.dt;
    float cmd_vel = current_speed;
    float stopping_dist = (current_speed * current_speed) / (2 * max_decel);
    // printf("dist_to_go: %f, current_speed: %f, stopping_dist: %f\n", dist_to_go, current_speed, stopping_dist);
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
float getFreePathLength(float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {
    // TODO: check that curvature is not 0
    // TODO: check negative angles for right turning
    Vector2f c = Vector2f(0, 1 / curvature);
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    float r_inner = c[1] - robot_config.width / 2;
    float r_tr = (Vector2f(0, r_inner) - Vector2f(h, 0)).norm();
    float r_tl = (c - Vector2f(h, 0)).norm();
    float r_br = (c - Vector2f(robot_config.base_link_offset, 0)).norm();
    float free_path_length = M_PI * c[1];  // some large number

    float theta_br = asin(robot_config.base_link_offset / r_br); // angle where back right would hit
    float phi = 0;

    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = atan2(p[0], c[1] - p[1]); // angle between p and c

        if (r_inner <= r_p && r_p <= r_tr) {    // inner side hit
            phi = acos(r_inner / r_p);
        }
        else if ((r_inner <= r_p && r_p <= r_br) and (-1*theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c[1] + robot_config.width / 2));
        }

        else if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
        }
        float length = (theta - phi) * r_p;
        free_path_length = std::min(free_path_length, length);
    }
    return free_path_length;
}


// sample path options
// given point cloud (robot frame), num options, max curvature
// out const vector path options

vector<navigation::PathOption> samplePathOptions(int num_options,
                                                    const vector<Eigen::Vector2f>& point_cloud,
                                                    const navigation::NavigationParams& robot_config) {
    static vector<navigation::PathOption> path_options;
    path_options.clear();
    float max_curvature = robot_config.max_curvature;

    // loop through curvature from max to -max
    for (int i = 0; i < num_options; i++) {
        float curvature = -1 * max_curvature + (2 * max_curvature) * i / (num_options - 1);
        float free_path_length = getFreePathLength(curvature, point_cloud, robot_config);
        navigation::PathOption path_option;
        path_option.curvature = curvature;
        path_option.free_path_length = free_path_length;
        path_options.push_back(path_option);
    }
    return path_options;
}

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
int selectPath(const vector<navigation::PathOption>& path_options) {
    int selected_path = 0;
    float max_free_path_length = 0;
    for (unsigned int i = 0; i < path_options.size(); i++) {
        if (path_options[i].free_path_length > max_free_path_length) {
            max_free_path_length = path_options[i].free_path_length;
            selected_path = i;
        }
    }
    return selected_path;
}

