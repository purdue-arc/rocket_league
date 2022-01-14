/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#ifndef CAMERA_TRACKING_TF_REBROADCASTER_H
#define CAMERA_TRACKING_TF_REBROADCASTER_H

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

namespace camera_tracking {

class TFRebroadcaster {
public:
  TFRebroadcaster(const ros::NodeHandle& nh, const std::string& subTopic,
                  const std::string& pubTopic, const std::string& prefix,
                  uint32_t queueSize);
  ~TFRebroadcaster();

private:
  void callback(tf2_msgs::TFMessageConstPtr msg);

  ros::NodeHandle _nh;
  std::string _prefix;

  ros::Subscriber _sub;
  ros::Publisher _pub;
};

}  // namespace camera_tracking

#endif  // CAMERA_TRACKING_TF_REBROADCASTER_H