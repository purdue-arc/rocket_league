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
CarLink car0{nh, "~car0/effort", 23};
CarLink car1{nh, "~car1/effort", 22};
CarLink car2{nh, "~car2/effort", 21};
CarLink car3{nh, "~car3/effort", 20};
CarLink car4{nh, "~car4/effort", 10};
CarLink car5{nh, "~car5/effort", 9};

// enabled / disable callback
void enable_callback(const std_msgs::Bool& enable) {
    if (enable.data) {
        car0.enable();
        car1.enable();
        car2.enable();
        car3.enable();
        car4.enable();
        car5.enable();
    } else {
        car0.disable();
        car1.disable();
        car2.disable();
        car3.disable();
        car4.disable();
        car5.disable();
    }
}
ros::Subscriber<std_msgs::Bool> enable_sub{"enable", enable_callback};


void setup() {
  // init node
  nh.initNode();
  nh.subscribe(enable_sub);

  // wait until connected
  while (!nh.connected()) {
    nh.spinOnce();
  }

  // get params
//   nh.getParam("~throttle_throw", &THROTTLE_THROW);

//   nh.getParam("~car0/max_left", &CAR0_MAX_LEFT);
//   nh.getParam("~car0/center", &CAR0_CENTER);
//   nh.getParam("~car0/max_right", &CAR0_MAX_RIGHT);

//   nh.getParam("~car1/max_left", &CAR1_MAX_LEFT);
//   nh.getParam("~car1/center", &CAR1_CENTER);
//   nh.getParam("~car1/max_right", &CAR1_MAX_RIGHT);

//   nh.getParam("~car2/max_left", &CAR2_MAX_LEFT);
//   nh.getParam("~car2/center", &CAR2_CENTER);
//   nh.getParam("~car2/max_right", &CAR2_MAX_RIGHT);

//   nh.getParam("~car3/max_left", &CAR3_MAX_LEFT);
//   nh.getParam("~car3/center", &CAR3_CENTER);
//   nh.getParam("~car3/max_right", &CAR3_MAX_RIGHT);

//   nh.getParam("~car4/max_left", &CAR4_MAX_LEFT);
//   nh.getParam("~car4/center", &CAR4_CENTER);
//   nh.getParam("~car4/max_right", &CAR4_MAX_RIGHT);

//   nh.getParam("~car5/max_left", &CAR5_MAX_LEFT);
//   nh.getParam("~car5/center", &CAR5_CENTER);
//   nh.getParam("~car5/max_right", &CAR5_MAX_RIGHT);
}

void loop() {
  nh.spinOnce();
}
