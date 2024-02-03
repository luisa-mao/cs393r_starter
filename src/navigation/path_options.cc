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
void setPathOption(navigation::PathOption& path_option,
                        float curvature, const vector<Eigen::Vector2f>& point_cloud,
                        const navigation::NavigationParams& robot_config) {
    path_option.curvature = curvature;
    float h = robot_config.length - robot_config.base_link_offset; // distance from base link to front bumper
    // TODO: check that curvature is not 0
    if (curvature == 0) {
        path_option.free_path_length = 10;  // some large number
        for (auto p: point_cloud) {
            if (curvature == 0 && robot_config.width/2 >= abs(p[1]) && h <= p[0]
                && p[0] < path_option.free_path_length) {
                path_option.free_path_length = p[0] - h;
                path_option.obstruction = p;
            }
        }
        return;
    }

    // TODO: check negative angles for right turning
    Vector2f c = Vector2f(0, 1 / curvature);
    float r_inner = c[1] - robot_config.width / 2;
    // float r_outer = c[1] + robot_config.width / 2;
    float r_tl = (Vector2f(0, r_inner) - Vector2f(h, 0)).norm();
    // float r_tr = (Vector2f(0, r_outer) - Vector2f(h, 0)).norm();
    // float r_br = (Vector2f(0, r_outer) - Vector2f(robot_config.base_link_offset, 0)).norm();
    path_option.free_path_length = std::min(M_PI * c[1], 10.0);  // some large number
    float omega = atan2(h, r_inner);


    // float theta_br = asin(robot_config.base_link_offset / r_br); // angle where back right would hit
    float phi = 0;

    for (unsigned int i = 0; i < point_cloud.size(); i++) {
        Vector2f p = point_cloud[i];
        float r_p = (c-p).norm();
        float theta = atan2(p[0], c[1] - p[1]); // angle between p and c
        float length = 10;
        if (theta < 0) {
            // theta += 2 * M_PI;
            continue;
        }

        if (r_inner <= r_p && r_p <= r_tl && theta >= omega) {    // inner side hit
            phi = acos(r_inner / r_p);
            // inner side hit
            std::cout << "Inner Hit: " << std::endl;
            std::cout << "Phi: " << phi << std::endl;
            length = (theta - phi) * r_p;

        }
        // else if ((r_inner <= r_p && r_p <= r_br) and (-1*theta_br <= theta && theta <= theta_br)) {    // outer side hit
        //     phi = acos(r_p / (c[1] + robot_config.width / 2));
        // }

        // else if (r_tl <= r_p && r_p <= r_tr) {    // front side hit
        //     phi = asin(h / r_p);
        // }
        // // std::cout << "theta: " << theta << ", phi: " << phi << ", r_p: " << r_p << std::endl;
        // // free_path_length = std::min(free_path_length, length);
        // std::cout << "Length: " << length << std::endl;

        if (length < path_option.free_path_length && length > 0) {
            path_option.free_path_length = length;
            path_option.obstruction = p;
        }
    }
    
    if (curvature == robot_config.max_curvature) {
        std::cout << "Obstruction: " << path_option.obstruction << std::endl;
        std::cout << "r_inner: " << r_inner << std::endl;
        std::cout << "r_tl: " << r_tl << std::endl;
        std::cout << "r_p: " << (c - path_option.obstruction).norm() << std::endl;
        std::cout << "free path length: " << path_option.free_path_length << std::endl;

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
    for (int i = 0; i < num_options; i++) {  // only do the positive ones for now
        // float curvature = -1 * max_curvature + (2 * max_curvature) * i / (num_options - 1);
        navigation::PathOption path_option;
        float curvature = max_curvature - (max_curvature) * i / (num_options - 1);
        setPathOption(path_option, curvature, point_cloud, robot_config);
        path_options.push_back(path_option);
    }

    // for each path in path_options, print the curvature and free path length
    for (unsigned int i = 0; i < path_options.size(); i++) {
        printf("curvature: %f, free path length: %f\n", path_options[i].curvature, path_options[i].free_path_length);
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

