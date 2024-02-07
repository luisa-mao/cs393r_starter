#include <queue>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "ros/ros.h"

#ifndef LATENCY_COMPENSATION_H
#define LATENCY_COMPENSATION_H

struct Control {
    float x_dot;
    float y_dot;
    float theta_dot;
    double time;
};

struct Observation {
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
    double time;
};

class LatencyCompensation {
public:
    LatencyCompensation(float actuation_latency, float observation_latency, double dt) :
        actuation_latency_(actuation_latency),
        observation_latency_(observation_latency),
        dt_(dt) {};
    
    // Mostly for debugging
    float getActuationDelay() {
        return actuation_latency_;
    }

    void setActuationDelay(float actuation_latency) {
        actuation_latency_ = actuation_latency;
    }

    float getObservationDelay() {
        return observation_latency_;
    }

    void setObservationDelay(float observation_latency) {
        observation_latency_ = observation_latency;
    }

    std::queue<Control> getControlQueue() {
        return control_queue_;
    }

    void recordControl(const Control& control) {
        control_queue_.push(control);
    }

    // Store last observed state
    void recordObservation(float x, float y, float theta, double time) {
        last_x_ = x;
        last_y_ = y;
        last_theta_ = theta;
        last_observation_time_ = time - observation_latency_;
    };

    Observation getPredictedState() {
        Observation predictedState = last_observation_;

        double control_cutoff_time_ = last_observation_.time - actuation_latency_;
        double current_time = ros::Time::now().toSec() - actuation_latency_;

        while (control_queue_.size() > 0 && control_queue_.front().time < control_cutoff_time_) 
            control_queue_.pop();

        bool current_found = false;
        while (control_queue_.size() > 0) {
            if (!current_found && control.time >= current_time) {
                predictedState.vx = control.x_dot;
                predictedState.vy = control.y_dot;
                predictedState.omega = control.theta_dot;
                current_found = true;
            }
            Control control = control_queue_.front();
            predictedState.x += control.x_dot * dt_;
            predictedState.y += control.y_dot * dt_;
            predictedState.theta += control.theta_dot * dt_;
            control_queue_.pop();
        }
        return predictedState;
    }

private:
    float actuation_latency_;
    float observation_latency_;
    double dt_;
    std::queue<Control> control_queue_;
    
    float last_x_;
    float last_y_;
    float last_theta_;
    double last_observation_time_;
};

#endif  // LATENCY_COMPENSATION_H