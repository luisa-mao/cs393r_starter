#include <queue>
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "ros/ros.h"

#ifndef LATENCY_COMPENSATION_H
#define LATENCY_COMPENSATION_H

struct Control {
    float velocity;
    float curvature;
    ros::Time time;
};

struct Observation;

class LatencyCompensation {
public:
    LatencyCompensation(float actuation_delay, float observation_delay) :
        actuation_delay_(actuation_delay),
        observable_delay_(observation_delay) {};
    
    float getActuationDelay() {
        return actuation_delay_;
    }

    void setActuationDelay(float actuation_delay) {
        actuation_delay_ = actuation_delay;
    }

    float getObservableDelay() {
        return observable_delay_;
    }

    void setObservableDelay(float observable_delay) {
        observable_delay_ = observable_delay;
    }

    std::queue<Control> getControlQueue() {
        return control_queue_;
    }

    void recordControl(const Control& control) {
        control_queue_.push(control);
        cout << "Control recorded at " << control.time << ": " << control.curvature << " " << control.velocity << endl;
    }

private:
    float actuation_delay_;
    float observable_delay_;
    std::queue<Control> control_queue_;
};

#endif  // LATENCY_COMPENSATION_H