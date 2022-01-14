/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/tf_rebroadcaster.h>

namespace camera_tracking {

TFRebroadcaster::TFRebroadcaster(const ros::NodeHandle& nh,
                                 const std::string& subTopic,
                                 const std::string& pubTopic,
                                 const std::string& prefix, uint32_t queueSize)
  : _nh(nh), _prefix(prefix) {
  _sub = _nh.subscribe(subTopic, queueSize, &TFRebroadcaster::callback, this);
  _pub = _nh.advertise<tf2_msgs::TFMessage>(pubTopic, queueSize);
}

TFRebroadcaster::~TFRebroadcaster() {
}

void TFRebroadcaster::callback(tf2_msgs::TFMessageConstPtr msg) {
  tf2_msgs::TFMessage tfOut;
  tfOut.transforms.reserve(msg->transforms.size());

  for (const auto& inTransform : msg->transforms) {
    geometry_msgs::TransformStamped transform = inTransform;
    if (_prefix != transform.header.frame_id) {
      transform.header.frame_id = _prefix + "_" + transform.header.frame_id;
    }
    if (_prefix != transform.child_frame_id) {
      transform.child_frame_id = _prefix + "_" + transform.child_frame_id;
    }
    tfOut.transforms.push_back(transform);
  }
  _pub.publish(tfOut);
}

}  // namespace camera_tracking
