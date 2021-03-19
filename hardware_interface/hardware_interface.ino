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
#include <PulsePositionIMXRT.h>
#include <PulsePosition.h>

// pins
const byte PPM_OUT_PIN = 23;

// channels
const byte THROTTLE_CHANNEL = 1;
const byte STEERING_CHANNEL = 2;


ros::NodeHandle nh;
PulsePositionOutput outputPPM(RISING);

//Setting Parameters
nh.setParam("THROTTLE_MAX_FORWARD", 2000);
nh.setParam("THROTTLE_ZERO", 1500);
nh.setParam("THROTTLE_MAX_BACKWARD", 1000);
nh.setParam("STEERING_MAX_RIGHT", 2000);
nh.setParam("STEERING_ZERO", 1500);
nh.setParam("STEERING_TRIM", 0);
nh.setParam("STEERING_MAX_LEFT", 1000);

//Getting Parameters
const int THROTTLE_MAX_FORWARD;
const int THROTTLE_ZERO;
const int THROTTLE_MAX_BACKWARD;
const int STEERING_MAX_RIGHT;
const int STEERING_ZERO;
const int STEERING_TRIM;
const int STEERING_MAX_LEFT;
nh.param("THROTTLE_MAX_FORWARD", THROTTLE_MAX_FORWARD, 2000);
nh.param("THROTTLE_ZERO", THROTTLE_ZERO, 1500);
nh.param("THROTTLE_MAX_BACKWARD", THROTTLE_MAX_BACKWARD, 1000);
nh.param("STEERING_MAX_RIGHT", STEERING_MAX_RIGHT, 2000);
nh.param("STEERING_ZERO", STEERING_ZERO, 1500);
nh.param("STEERING_TRIM", 0);
nh.param("STEERING_MAX_LEFT", STEERING_MAX_LEFT, 1000);


// Timing Parameters for PulsePosition Library
// *********************DEFAULT VALUES******************
// Minimum Time between Rising Edges: 300.0 microseconds
// Maximum Time between Rising Edges: 2500.0 microseconds
// *****************************************************
const float effort_to_ppm_throttle(const float effort)
{
  return THROTTLE_ZERO + effort * 500;
}
const float effort_to_ppm_steering(const float effort)
{
  return THROTTLE_ZERO + (effort + STEERING_TRIM) * 500;
}
void throttle_callback(const std_msgs::Float32& throttle)
{
  outputPPM.write(THROTTLE_CHANNEL, effort_to_ppm_throttle(throttle.data));
}

void steering_callback(const std_msgs::Float32& steering)
{
  outputPPM.write(STEERING_CHANNEL, effort_to_ppm_steering(steering.data));
}

ros::Subscriber<std_msgs::Float32> throttle_sub("control_effort/throttle", throttle_callback);
ros::Subscriber<std_msgs::Float32> steering_sub("control_effort/steering", steering_callback);

void setup()
{
  Serial.begin(57600);
  pinMode(PPM_OUT_PIN, OUTPUT);
  digitalWrite(PPM_OUT_PIN, LOW);
  
  // ros
  nh.initNode();
  nh.subscribe(throttle_sub);
  nh.subscribe(steering_sub);

  // begin sending PPM signal
  outputPPM.begin(PPM_OUT_PIN);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
