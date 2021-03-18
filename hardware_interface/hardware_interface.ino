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
const int THROTTLE_CHANNEL = 1;
const int STEERING_CHANNEL = 2;


ros::NodeHandle nh;
PulsePositionOutput outputPPM(RISING);


// Timing Parameters for PulsePosition Library
// *********************DEFAULT VALUES******************
// Minimum Time between Rising Edges: 300.0 microseconds
// Maximum Time between Rising Edges: 3500.0 microseconds
// *****************************************************
const float effort_to_ppm_duration(const float effort)
{
  return effort;
}
const byte effort_to_pwm(const float effort, const byte MIN_PWN, const byte MID_PWM, const byte MAX_PWM)
{
  if(effort >= 0)
  {
    return (byte)(MID_PWM + effort * (MAX_PWM - MID_PWM));
  }
  else
  {
    return (byte)(MID_PWM + effort * (MID_PWM - MIN_PWN));
  }
}

void throttle_callback(const std_msgs::Float32& throttle)
{
  outputPPM.write(THROTTLE_CHANNEL, effort_to_ppm_duration(throttle.data));
}

void steering_callback(const std_msgs::Float32& steering)
{
  outputPPM.write(STEERING_CHANNEL, effort_to_ppm_duration(steering.data));
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
