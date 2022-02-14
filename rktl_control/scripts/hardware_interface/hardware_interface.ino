/*********************************************************************
# Copyright (c) 2021, Autonomous Robotics Club of Purdue
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros.h>
#include <std_msgs/Bool.h>
#include <rktl_msgs/ControlEffort.h>
#include <PulsePosition.h>

// pins
const byte PPM_OUT_PIN0 = 23;
const byte PPM_OUT_PIN1 = 22;
const byte PPM_OUT_PIN2 = 21;
const byte PPM_OUT_PIN3 = 20;
const byte PPM_OUT_PIN4 = 10;
const byte PPM_OUT_PIN5 = 9;

// channels
const byte THROTTLE_CHANNEL = 1;
const byte STEERING_CHANNEL = 2;

ros::NodeHandle nh;
PulsePositionOutput outputPPM0(RISING);
PulsePositionOutput outputPPM1(RISING);
PulsePositionOutput outputPPM2(RISING);
PulsePositionOutput outputPPM3(RISING);
PulsePositionOutput outputPPM4(RISING);
PulsePositionOutput outputPPM5(RISING);

// Parameters
float THROTTLE_LIMIT = 1.0;
int THROTTLE_ZERO = 1500;
int THROTTLE_THROW = 500;
int CAR0_MAX_LEFT = 1000;
int CAR0_CENTER = 1500;
int CAR0_MAX_RIGHT = 2000;
int CAR1_MAX_LEFT = 1000;
int CAR1_CENTER = 1500;
int CAR1_MAX_RIGHT = 2000;
int CAR2_MAX_LEFT = 1000;
int CAR2_CENTER = 1500;
int CAR2_MAX_RIGHT = 2000;
int CAR3_MAX_LEFT = 1000;
int CAR3_CENTER = 1500;
int CAR3_MAX_RIGHT = 2000;
int CAR4_MAX_LEFT = 1000;
int CAR4_CENTER = 1500;
int CAR4_MAX_RIGHT = 2000;
int CAR5_MAX_LEFT = 1000;
int CAR5_CENTER = 1500;
int CAR5_MAX_RIGHT = 2000;

// Variables
bool enabled = true;

void enable_callback(const std_msgs::Bool& enable) {
  enabled = enable.data;
}

const float effort_to_ppm_throttle(const float effort) {
  if (enabled) {
    return THROTTLE_ZERO + effort * THROTTLE_THROW * THROTTLE_LIMIT;
  }
  else {
    return THROTTLE_ZERO;
  }
}

const float effort_to_ppm_steering(const float effort, const int left, const int center, const int right) {
  if (effort > 0) {
    return (left - center) * effort + center;
  }
  else if (effort < 0) {
    return (center - right) * effort + center;
  }
  else {
    return center;
  }
}

void control_callback0(const rktl_msgs::ControlEffort& control) {
  outputPPM0.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM0.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR0_MAX_LEFT, CAR0_CENTER, CAR0_MAX_RIGHT));
}

void control_callback1(const rktl_msgs::ControlEffort& control) {
  outputPPM1.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM1.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR1_MAX_LEFT, CAR1_CENTER, CAR1_MAX_RIGHT));
}

void control_callback2(const rktl_msgs::ControlEffort& control) {
  outputPPM2.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM2.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR2_MAX_LEFT, CAR2_CENTER, CAR2_MAX_RIGHT));
}

void control_callback3(const rktl_msgs::ControlEffort& control) {
  outputPPM3.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM3.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR3_MAX_LEFT, CAR3_CENTER, CAR3_MAX_RIGHT));
}

void control_callback4(const rktl_msgs::ControlEffort& control) {
  outputPPM4.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM4.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR4_MAX_LEFT, CAR4_CENTER, CAR4_MAX_RIGHT));
}

void control_callback5(const rktl_msgs::ControlEffort& control) {
  outputPPM5.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle));
  outputPPM5.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering, CAR5_MAX_LEFT, CAR5_CENTER, CAR5_MAX_RIGHT));
}

ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub0("car0/effort", control_callback0);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub1("car1/effort", control_callback1);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub2("car2/effort", control_callback2);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub3("car3/effort", control_callback3);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub4("car4/effort", control_callback4);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub5("car5/effort", control_callback5);
ros::Subscriber<std_msgs::Bool> enable_sub("enable", enable_callback);

void setup() {
  // init node
  nh.initNode();

  // get params
  nh.getParam("~throttle_limit", &THROTTLE_LIMIT);

  nh.getParam("~car0/max_left", &CAR0_MAX_LEFT);
  nh.getParam("~car0/center", &CAR0_CENTER);
  nh.getParam("~car0/max_right", &CAR0_MAX_RIGHT);

  nh.getParam("~car1/max_left", &CAR1_MAX_LEFT);
  nh.getParam("~car1/center", &CAR1_CENTER);
  nh.getParam("~car1/max_right", &CAR1_MAX_RIGHT);

  nh.getParam("~car2/max_left", &CAR2_MAX_LEFT);
  nh.getParam("~car2/center", &CAR2_CENTER);
  nh.getParam("~car2/max_right", &CAR2_MAX_RIGHT);

  nh.getParam("~car3/max_left", &CAR3_MAX_LEFT);
  nh.getParam("~car3/center", &CAR3_CENTER);
  nh.getParam("~car3/max_right", &CAR3_MAX_RIGHT);

  nh.getParam("~car4/max_left", &CAR4_MAX_LEFT);
  nh.getParam("~car4/center", &CAR4_CENTER);
  nh.getParam("~car4/max_right", &CAR4_MAX_RIGHT);

  nh.getParam("~car5/max_left", &CAR5_MAX_LEFT);
  nh.getParam("~car5/center", &CAR5_CENTER);
  nh.getParam("~car5/max_right", &CAR5_MAX_RIGHT);

  // subscribers
  nh.subscribe(control_effort_sub0);
  nh.subscribe(control_effort_sub1);
  nh.subscribe(control_effort_sub2);
  nh.subscribe(control_effort_sub3);
  nh.subscribe(control_effort_sub4);
  nh.subscribe(control_effort_sub5);

  // initialize to no motion commands
  outputPPM0.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM0.write(STEERING_CHANNEL, CAR0_CENTER);
  outputPPM1.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM1.write(STEERING_CHANNEL, CAR1_CENTER);
  outputPPM2.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM2.write(STEERING_CHANNEL, CAR2_CENTER);
  outputPPM3.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM3.write(STEERING_CHANNEL, CAR3_CENTER);
  outputPPM4.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM4.write(STEERING_CHANNEL, CAR4_CENTER);
  outputPPM5.write(THROTTLE_CHANNEL, THROTTLE_ZERO);
  outputPPM5.write(STEERING_CHANNEL, CAR5_CENTER);

  // begin sending PPM signal
  outputPPM0.begin(PPM_OUT_PIN0);
  outputPPM1.begin(PPM_OUT_PIN1);
  outputPPM2.begin(PPM_OUT_PIN2);
  outputPPM3.begin(PPM_OUT_PIN3);
  outputPPM4.begin(PPM_OUT_PIN4);
  outputPPM5.begin(PPM_OUT_PIN5);
}

void loop() {
  nh.spinOnce();
}
