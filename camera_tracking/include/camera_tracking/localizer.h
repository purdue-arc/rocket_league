#ifndef CAMERA_TRACKING_TF_REBROADCASTER_H
#define CAMERA_TRACKING_TF_LOCALIZER_H

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <Eigen/Dense>

namespace camera_tracking {

class Localizer {
public:
  Localizer(const ros::NodeHandle& nh, const std::string& detectionTopic,
            const std::string& originId, const std::string& pubTopic,
            const std::map<std::string, std::string>& pubTopics, int bufferSize, int queueSize);
  ~Localizer() = default;

  static std::string idsToString(std::vector<int> ids);

private:
  void callback(apriltag_ros::AprilTagDetectionArrayConstPtr msg);
  Eigen::Matrix4d combineMatrices(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos);
  geometry_msgs::PoseWithCovarianceStamped toMsg(const Eigen::Matrix4d& transform, ros::Time stamp,
                                                 const std::string& frameId = "map");

  ros::NodeHandle _nh;
  ros::Subscriber _sub;
  std::string _originId;
  ros::Publisher _pub;
  std::map<std::string, ros::Publisher> _pubs;
  int _bufferSize;
  int _bufferPos;
  std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> _buffer;
};

}  // namespace camera_tracking

#endif  // CAMERA_TRACKING_TF_LOCALIZER_H
