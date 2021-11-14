#include <camera_tracking/localizer.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  std::string detectionTopic;
  std::string originId;
  std::map<std::string, std::string> pubTopics;
  int bufferSize;
  int queueSize;

  ros::param::param<std::string>("~dectection_topic", detectionTopic,
                                 "tag_detections");
  ros::param::param<std::string>("~origin_id", originId, "0");
  ros::param::get("~pub_topics", pubTopics);
  ros::param::param<int>("~buffer_size", bufferSize, 10);
  ros::param::param<int>("~queue_size", queueSize, 100);

  camera_tracking::Localizer localizer(nh, detectionTopic, originId, pubTopics,
                                       bufferSize, queueSize);

  ros::spin();
  return 0;
}
