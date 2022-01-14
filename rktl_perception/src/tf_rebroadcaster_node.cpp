/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/tf_rebroadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tf_rebroadcaster");
  ros::NodeHandle nh;

  std::string prefix;
  int queueSize;

  ros::param::param<std::string>("~prefix", prefix, "foo");
  ros::param::param<int>("~queue_size", queueSize, 100);

  camera_tracking::TFRebroadcaster tfrb(nh, "tf", "/tf", prefix, queueSize);
  camera_tracking::TFRebroadcaster tfrbStatic(nh, "tf_static", "/tf_static",
                                              prefix, queueSize);

  ros::spin();
  return 0;
}
