#include <camera_tracking/localizer.h>
#include <Eigen/Dense>

namespace camera_tracking {

Localizer::Localizer(const ros::NodeHandle& nh,
                     const std::string& detectionTopic,
                     const std::string& originId,
                     const std::map<std::string, std::string>& pubTopics,
                     int bufferSize, int queueSize)
  : _nh(nh), _originId(originId) {
  _sub = _nh.subscribe(detectionTopic, queueSize, &Localizer::callback, this);
  for (auto& kv : pubTopics) {
    _pubs[kv.first] = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        kv.second, queueSize);
  }
}

std::string Localizer::idsToString(std::vector<int> ids) {
  if (ids.size() == 0)
    return "";

  std::sort(ids.begin(), ids.end());
  std::ostringstream oss;
  oss << ids.front();
  for (auto it = ids.begin() + 1; it != ids.end(); ++it)
    oss << ',' << *it;
  return oss.str();
}

void Localizer::callback(apriltag_ros::AprilTagDetectionArrayConstPtr msg) {
  Eigen::Matrix3d baseRotation;
  std::map<std::string, Eigen::Matrix4d> transforms;

  for (auto& detection : msg->detections) {
    std::string idStr = idsToString(detection.id);
    geometry_msgs::Quaternion rot = detection.pose.pose.pose.orientation;
    geometry_msgs::Point pos = detection.pose.pose.pose.position;
    Eigen::Quaterniond quat(rot.w, rot.x, rot.y, rot.z);

    if (idStr == _originId) {
      baseRotation = quat.toRotationMatrix();
    } else if (_pubs.find(idStr) != _pubs.end()) {
      transforms[idStr].block(0, 0, 3, 3) << quat.toRotationMatrix();
      transforms[idStr].col(3) << pos.x, pos.y, pos.z, 1.0;
      transforms[idStr].row(3).head(3) << 0.0, 0.0, 0.0;
    } else {
      ROS_INFO("Detected tag with unknown id: %s", idStr.c_str());
    }
  }
}

}  // namespace camera_tracking
