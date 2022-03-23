/*********************************************************************
Teensyduino sketch used to control six cars at once over six radio links.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#include <ros.h>
#include <std_msgs/Bool.h>

#include "CarLink.hpp"

// ros node handle
ros::NodeHandle nh;

// radio links to each car
CarLink car0{&nh, "~car0/", 23};
CarLink car1{&nh, "~car1/", 22};
CarLink car2{&nh, "~car2/", 21};
CarLink car3{&nh, "~car3/", 20};
CarLink car4{&nh, "~car4/", 10};
CarLink car5{&nh, "~car5/", 9};

// helper functions
void enable_all() {
    car0.enable();
    car1.enable();
    car2.enable();
    car3.enable();
    car4.enable();
    car5.enable();
}

void disable_all() {
    car0.disable();
    car1.disable();
    car2.disable();
    car3.disable();
    car4.disable();
    car5.disable();
}

bool update_all() {
    return car0.update_params()
        && car1.update_params()
        && car2.update_params()
        && car3.update_params()
        && car4.update_params()
        && car5.update_params();
}

// enabled / disable callback and subscriber
void enable_callback(const std_msgs::Bool& enable) {
    if (enable.data) {
        enable_all();
    } else {
       disable_all();
    }
}
ros::Subscriber<std_msgs::Bool> enable_sub{"enable", enable_callback};

// flag for updating params on comms loss
bool comms_loss = false;

void setup() {
    // init node
    nh.initNode();

    // wait until connected
    while (!nh.connected()) {
        nh.spinOnce();
    }

    // get params
    update_all();

    // allow movement
    nh.subscribe(enable_sub);
}

void loop() {
    if (!nh.connected()) {
        disable_all();
        comms_loss = true;
    } else if (comms_loss) {
        update_all();
        comms_loss = false;
    }

    nh.spinOnce();
}
