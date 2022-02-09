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
#include <std_msgs/Float32.h>
//#include <PulsePositionIMXRT.h>
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

//Getting Parameters
Float32 THROTTLE_LIMIT = 1;
int THROTTLE_MAX_FORWARD = 2000;
int THROTTLE_ZERO = 1500;
int THROTTLE_MAX_BACKWARD = 1000;
int STEERING_MAX_RIGHT = 2000;
int STEERING_ZERO = 1500;
int STEERING_MAX_LEFT = 1000;
int CAR0_TRIM = 0;
int CAR1_TRIM = 0;
int CAR2_TRIM = 0;
int CAR3_TRIM = 0;
int CAR4_TRIM = 0;
int CAR5_TRIM = 0;
int CAR0_STEERING = 1;
int CAR1_STEERING = 1;
int CAR2_STEERING = 1;
int CAR3_STEERING = 1;
int CAR4_STEERING = 1;
int CAR5_STEERING = 1;





void enable_callbacks(const std_msgs::Bool& enable)
{
  if (enable.data) {
    nh.getParam("THROTTLE_LIMIT", &THROTTLE_LIMIT, 1);
  } 
  else {
    THROTTLE_LIMIT = 0;
  }
}

// Timing Parameters for PulsePosition Library
// *********************DEFAULT VALUES******************
// Minimum Time between Rising Edges: 300.0 microseconds
// Maximum Time between Rising Edges: 2500.0 microseconds
// *****************************************************
const float effort_to_ppm_throttle(const float effort)
{
  return THROTTLE_ZERO + effort * 500 * THROTTLE_LIMIT;
}

const float effort_to_ppm_steering(const float effort, const int trim)
{
  return STEERING_ZERO + (effort + trim) * 500;
}

void control_callback0(const rktl_msgs::ControlEffort& control)
{
  outputPPM0.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM0.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR0_STEERING, CAR0_TRIM));
}

void control_callback1(const rktl_msgs::ControlEffort& control)
{
  outputPPM1.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM1.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR1_STEERING, CAR1_TRIM));
}

void control_callback2(const rktl_msgs::ControlEffort& control)
{
  outputPPM2.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM2.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR2_STEERING, CAR2_TRIM));
}

void control_callback3(const rktl_msgs::ControlEffort& control)
{
  outputPPM3.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM3.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR3_STEERING, CAR3_TRIM));
}

void control_callback4(const rktl_msgs::ControlEffort& control)
{
  outputPPM4.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM4.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR4_STEERING, CAR4_TRIM));
}

void control_callback5(const rktl_msgs::ControlEffort& control)
{
  outputPPM5.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(control.throttle.data));
  outputPPM5.write(STEERING_CHANNEL, effort_to_ppm_steering(control.steering.data * CAR5_STEERING, CAR5_TRIM));
}

//Might have to change subscriber name
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub0("car0/control_effort", control_callback0);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub1("car1/control_effort", control_callback1);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub2("car2/control_effort", control_callback2);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub3("car3/control_effort", control_callback3);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub4("car4/control_effort", control_callback4);
ros::Subscriber<rktl_msgs::ControlEffort> control_effort_sub5("car5/control_effort", control_callback5);
ros::Subscriber<std_msgs::Bool> enable_sub("enable", enable_callback);

void setup()
{
  Serial.begin(57600);
  pinMode(PPM_OUT_PIN0, OUTPUT);
  pinMode(PPM_OUT_PIN1, OUTPUT);
  pinMode(PPM_OUT_PIN2, OUTPUT);
  pinMode(PPM_OUT_PIN3, OUTPUT);
  pinMode(PPM_OUT_PIN4, OUTPUT);
  pinMode(PPM_OUT_PIN5, OUTPUT);
  //digitalWrite(PPM_OUT_PIN, LOW);
  
  // ros
  nh.initNode();
  
  nh.getParam("THROTTLE_LIMIT", &THROTTLE_LIMIT, 1);
  nh.getParam("CAR0_TRIM", &CAR0_TRIM, 0);
  nh.getParam("CAR1_TRIM", &CAR1_TRIM, 0);
  nh.getParam("CAR2_TRIM", &CAR2_TRIM, 0);
  nh.getParam("CAR3_TRIM", &CAR3_TRIM, 0);
  nh.getParam("CAR4_TRIM", &CAR4_TRIM, 0);
  nh.getParam("CAR5_TRIM", &CAR5_TRIM, 0);
  nh.getParam("CAR0_STEERING", &CAR0_STEERING, 1);
  nh.getParam("CAR1_STEERING", &CAR1_STEERING, 1);
  nh.getParam("CAR2_STEERING", &CAR2_STEERING, 1);
  nh.getParam("CAR3_STEERING", &CAR3_STEERING, 1);
  nh.getParam("CAR4_STEERING", &CAR4_STEERING, 1);
  nh.getParam("CAR5_STEERING", &CAR5_STEERING, 1);

  nh.subscribe(control_effort_sub0);
  nh.subscribe(control_effort_sub1);
  nh.subscribe(control_effort_sub2);
  nh.subscribe(control_effort_sub3);
  nh.subscribe(control_effort_sub4);
  nh.subscribe(control_effort_sub5);
 

  // begin sending PPM signal
  outputPPM0.begin(PPM_OUT_PIN0);
  outputPPM1.begin(PPM_OUT_PIN1);
  outputPPM2.begin(PPM_OUT_PIN2);
  outputPPM3.begin(PPM_OUT_PIN3);
  outputPPM4.begin(PPM_OUT_PIN4);
  outputPPM5.begin(PPM_OUT_PIN5);
}

void loop()
{
  nh.spinOnce();
}