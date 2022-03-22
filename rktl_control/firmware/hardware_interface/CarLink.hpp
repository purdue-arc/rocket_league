/*********************************************************************
Class definition for a single car's radio link.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#pragma once

#include <PulsePosition.h>

#include <ros.h>
#include <rktl_msgs/ControlEffort.h>

class CarLink {
    public:
    CarLink(const ros::NodeHandle& nh, const char *const topic, const int pin);
    ~CarLink();

    int throttle_throw;
    int steering_center;
    int steering_left;
    int steering_right;

    void enable();
    void disable();

    private:
    const int THROTTLE_ZERO = 1500;
    const int THROTTLE_CHANNEL = 1;
    const int STEERING_CHANNEL = 2;

    bool enabled;
    PulsePositionOutput ppm_out;
    ros::Subscriber<rktl_msgs::ControlEffort, CarLink> effort_sub;

    void effort_cb(const rktl_msgs::ControlEffort& effort);
};
