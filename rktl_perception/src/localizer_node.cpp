/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/localizer.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  std::string detectionTopic;
  std::string originId;
  std::string pubTopic;
  std::map<std::string, std::string> pubTopics;
  std::string ballSubTopic;
  std::string ballPubTopic;
  double ballRadius;
  int bufferSize;
  int queueSize;

  ros::param::param<std::string>("~dectection_topic", detectionTopic, "tag_detections");
  ros::param::param<std::string>("~origin_id", originId, "0");
  ros::param::get("~pub_topic", pubTopic);
  ros::param::get("~pub_topics", pubTopics);
  ros::param::param<std::string>("~ball_sub_topic", ballSubTopic, "ball_vector");
  ros::param::param<std::string>("~ball_pub_topic", ballPubTopic, "ball_pos");
  ros::param::param<double>("~ball_radius", ballRadius, 0.05);
  ros::param::param<int>("~buffer_size", bufferSize, 10);
  ros::param::param<int>("~queue_size", queueSize, 100);

  camera_tracking::Localizer localizer(nh, detectionTopic, originId, pubTopic, pubTopics,
                                       ballSubTopic, ballPubTopic, ballRadius, bufferSize,
                                       queueSize);

  ros::spin();
  return 0;
}
