/*********************************************************************
Teensyduino sketch used to control six cars at once over six radio links.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
*********************************************************************/

#include <ros.h>
#include <std_msgs/Bool.h>

#include "CarLink.hpp"

// ros node handle
ros::NodeHandle nh;

// radio links to each car
CarLink car0{&nh, 0, 23};
CarLink car1{&nh, 1, 22};
CarLink car2{&nh, 2, 21};
CarLink car3{&nh, 3, 20};
CarLink car4{&nh, 4, 10};
CarLink car5{&nh, 5, 9};

// helper functions
void enable_all() {
    car0.enable();
    car1.enable();
    car2.enable();
    car3.enable();
    car4.enable();
    car5.enable();
    digitalWrite(LED_BUILTIN, HIGH);
}

void disable_all() {
    car0.disable();
    car1.disable();
    car2.disable();
    car3.disable();
    car4.disable();
    car5.disable();
    digitalWrite(LED_BUILTIN, LOW);
}

bool update_all() {
    return car0.update_params()
        && car1.update_params()
        && car2.update_params()
        && car3.update_params()
        && car4.update_params()
        && car5.update_params();
}

// flag for if params have been properly set
bool configured = false;

// enabled / disable callback and subscriber
void enable_callback(const std_msgs::Bool& enable) {
    if (enable.data && configured) {
        enable_all();
    } else {
       disable_all();
    }
}

// subscriber object
ros::Subscriber<std_msgs::Bool> enable_sub{"enable", enable_callback};

void setup() {
    // LED pin
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // init node
    nh.initNode();
    nh.subscribe(enable_sub);

    // wait until connected
    while (!nh.connected()) {
        nh.spinOnce();
    }

    // get params
    configured = update_all();
}

void loop() {
    if (!nh.connected()) {
        configured = false;
        disable_all();
    } else if (!configured) {
        configured = update_all();
    }

    nh.spinOnce();
}
