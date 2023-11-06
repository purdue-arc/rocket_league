/*********************************************************************
Class implementation for a single car's radio link.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#include "CarLink.hpp"

CarLink::CarLink(ros::NodeHandle *const nh, const int car_num, const int pin) :
        nh{nh},
        car_num{car_num},
        topic{String("car") + String(car_num) + String("/effort")},
        throttle_throw{0},
        steering_center{1500},
        steering_left{1500},
        steering_right{1500},
        enabled{false},
        ppm_out{RISING},
        effort_sub{topic.c_str(), &CarLink::effort_cb, this} {
    ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
    ppm_out.write(STEERING_CHANNEL, steering_center);
    ppm_out.begin(pin);
    this->nh->subscribe(effort_sub);
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

bool CarLink::update_params() {
    if (nh->connected()
            && nh->getParam((String("~car") + String(car_num) + String("/throttle_throw")).c_str(),    &throttle_throw)
            && nh->getParam((String("~car") + String(car_num) + String("/steering_center")).c_str(),   &steering_center)
            && nh->getParam((String("~car") + String(car_num) + String("/steering_left")).c_str(),     &steering_left)
            && nh->getParam((String("~car") + String(car_num) + String("/steering_right")).c_str(),    &steering_right)) {
        return true;
    } else {
        throttle_throw = 0;
        steering_center = 1500;
        steering_left = 1500;
        steering_right = 1500;
        return false;
    }
}

void CarLink::effort_cb(const rktl_msgs::ControlEffort& effort) {
    // throttle
    if (enabled) {
        ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO + throttle_throw * effort.throttle);
    } else {
        ppm_out.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
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
