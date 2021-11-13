#ifndef CAMERA_TRACKING_TF_REBROADCASTER_H
#define CAMERA_TRACKING_TF_LOCALIZER_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

namespace camera_tracking {

class Localizer {
public:
  Localizer(const ros::NodeHandle& nh, const std::string& detectionTopic,
            const std::string& originId,
            const std::map<std::string, std::string>& pubTopics, int bufferSize,
            int queueSize);
  ~Localizer() = default;

  static std::string idsToString(std::vector<int> ids);

private:
  void callback(apriltag_ros::AprilTagDetectionArrayConstPtr msg);

  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  std::map<std::string, ros::Publisher> _pubs;
  std::string _originId;
};

}  // namespace camera_tracking

#endif  // CAMERA_TRACKING_TF_LOCALIZER_H
