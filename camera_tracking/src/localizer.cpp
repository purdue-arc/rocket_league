#include <camera_tracking/localizer.h>
#include <boost/numeric/ublas/matrix.hpp>

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
  for (auto& detection : msg->detections) {
    std::string idStr = idsToString(detection.id);
    if (idStr == _originId) {
      // TODO
    } else if (_pubs.find(idStr) != _pubs.end()) {
      ros::Publisher& pub = _pubs[idStr];
      pub.publish(detection.pose);
    } else {
      ROS_INFO("Detected tag with unknown id: %s", idStr.c_str());
    }
  }
}

}  // namespace camera_tracking
