/*********************************************************************
Class implementation for a single car's radio link.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#include "CarLink.hpp"

CarLink::CarLink(const ros::NodeHandle& nh, const char *const topic, const int pin) :
        throttle_throw{0},
        steering_center{1500},
        steering_left{1500},
        steering_right{1500},
        enabled{false},
        ppm_out{RISING},
        effort_sub{topic, &CarLink::effort_cb, this} {
    ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
    ppm_out.write(STEERING_CHANNEL, steering_center);
    ppm_out.begin(pin);
    nh.subscribe(effort_sub);
}

CarLink::~CarLink() {
    disable();
}

void CarLink::enable() {
    enabled = true;
}

void CarLink::disable() {
    ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
    enabled = false;
}

void CarLink::effort_cb(const rktl_msgs::ControlEffort& effort) {
    // throttle
    if (enabled) {
        ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
    } else {
        ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO + throttle_throw * effort.throttle);
    }

    // steering
    if (effort.steering > 0) {
        ppm_out.write(STEERING_CHANNEL, steering_center + (steering_left - steering_center) * effort.steering);
    } else if (effort.steering < 0) {
        ppm_out.write(STEERING_CHANNEL, steering_center + (steering_center - steering_right) * effort.steering);
    } else {
        ppm_out.write(STEERING_CHANNEL, steering_center);
    }
}
