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
    float cruise_stopping_dist = pow(current_speed, 2) / (2 * max_decel);
    float accel_stopping_dist = pow(current_speed + dt * max_accel, 2) / (2 * max_decel);
    // std::cout << "Current Speed: " << current_speed << std::endl;
    // std::cout << "Max Velocity: " << max_vel << std::endl;
    // std::cout << "dt: " << dt << std::endl;
    // std::cout << "Cruise Stopping Distance: " << cruise_stopping_dist << std::endl;
    // std::cout << "Dist to go: " << dist_to_go << std::endl;

    // if dist_to_go is larger than stopping_dist and not at max vel, can accelerate
    if (dist_to_go > accel_stopping_dist && current_speed < max_vel) {
        return current_speed + max_accel * dt;
    }
    else if (dist_to_go > cruise_stopping_dist && current_speed == max_vel) {  // can stop in time and at max vel
                                                                        // probably needs hysteresis
        return current_speed;
    }
    else {  // otherwise needs to decelerate
        return std::max(current_speed - max_decel * dt, 0.0f);
    }
}



// set curvature, free path length, obstruction for a path option
void setPathOption(navigation::PathOption& path_option,
                        float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {
    path_option.curvature = curvature;
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    if (curvature == 0) {
        path_option.free_path_length = 20;  // some large number
        for (auto p: point_cloud) {
            if (curvature == 0 && robot_config.width/2 >= abs(p[1]) && 0 <= p[0]
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h;
                path_option.obstruction = p;
            }
        }
        return;
    }

    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c.norm() - robot_config.width / 2;
    float r_outer = c.norm() + robot_config.width / 2;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h, 0)).norm();
    float r_tr = (Vector2f(0, r_outer) - Vector2f(h, 0)).norm();
    float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c.norm(), 20.0);  // some large number
    // float omega = atan2(h, r_inner);

    float theta_br = asin(robot_config.base_link_offset / r_br); // angle where back right would hit
    float phi = 0;

    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = curvature < 0 ? atan2(p[0], p[1]- c[1]) : atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 20;
        if (r_inner <= r_p && r_p <= r_tl) {    // inner side hit
            phi = acos(r_inner / r_p);
            length = (theta - phi) * c.norm();
        }
        else if ((r_inner <= r_p && r_p <= r_br) && (-theta_br <= theta && theta <= theta_br)) {    // outer side hit
            phi = acos(r_p / (c.norm() + robot_config.width / 2));
            length = (theta - phi) * c.norm();
        }

        else if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
            phi = asin(h / r_p);
            length = (theta - phi) * c.norm();
        }
        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
        }
    }
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
        float curvature = max_curvature * pow(2*i/float(num_options-1) - 1, 2);
        if (i < num_options / 2) {
            curvature = -curvature;
        }
        
        navigation::PathOption path_option;
        setPathOption(path_option, curvature, point_cloud, robot_config);
        path_options.push_back(path_option);
    }
    // exit(0);
    return path_options;
}

// returns the index of the selected path
// for now, just return the index of the path with the longest free path length
// if there are multiple paths with the same free path length, return the one with the smallest curvature
int selectPath(const vector<navigation::PathOption>& path_options) {
    int selected_path = 0;
    float max_free_path_length = 0;
    float min_curvature = 100; // some large number
    for (unsigned int i = 0; i < path_options.size(); i++) {
        if (path_options[i].free_path_length > max_free_path_length || 
            (path_options[i].free_path_length == max_free_path_length && abs(path_options[i].curvature) < min_curvature)){
            max_free_path_length = path_options[i].free_path_length;
            min_curvature = abs(path_options[i].curvature);
            selected_path = i;
        }
    }
    return selected_path;
}

