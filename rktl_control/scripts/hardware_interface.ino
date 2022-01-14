/* Interface to radio hardware.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

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

//Getting Parameters
int THROTTLE_MAX_FORWARD = 2000;
int THROTTLE_ZERO = 1500;
int THROTTLE_MAX_BACKWARD = 1000;
int STEERING_MAX_RIGHT = 2000;
int STEERING_ZERO = 1500;
int STEERING_TRIM = 0;
int STEERING_MAX_LEFT = 1000;


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
  return STEERING_ZERO + (effort + STEERING_TRIM) * 500;
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
  nh.getParam("THROTTLE_MAX_FORWARD", &THROTTLE_MAX_FORWARD, 2000);
  nh.getParam("THROTTLE_ZERO", &THROTTLE_ZERO, 1500);
  nh.getParam("THROTTLE_MAX_BACKWARD", &THROTTLE_MAX_BACKWARD, 1000);
  nh.getParam("STEERING_MAX_RIGHT", &STEERING_MAX_RIGHT, 2000);
  nh.getParam("STEERING_ZERO", &STEERING_ZERO, 1500);
  nh.getParam("STEERING_TRIM", &STEERING_TRIM, 0);
  nh.getParam("STEERING_MAX_LEFT", &STEERING_MAX_LEFT, 1000);
  nh.subscribe(throttle_sub);
  nh.subscribe(steering_sub);

  // begin sending PPM signal
  outputPPM.begin(PPM_OUT_PIN);
}

void loop()
{
  nh.spinOnce();
}
