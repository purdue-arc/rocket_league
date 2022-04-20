/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/localizer.h>

namespace camera_tracking {

Localizer::Localizer(const ros::NodeHandle& nh, const std::string& detectionTopic,
                     const std::string& originId, const std::string& pubTopic,
                     const std::map<std::string, std::string>& pubTopics,
                     const std::string& ballSubTopic, const std::string& ballPubTopic,
                     double ballRadius, int bufferSize, int queueSize)
  : _nh(nh), _originId(originId), _ballRadius(ballRadius), _bufferSize(bufferSize), _bufferPos(0) {
  _sub = _nh.subscribe(detectionTopic, queueSize, &Localizer::apriltagCallback, this);
  _pub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(pubTopic, queueSize);
  _ballSub = _nh.subscribe(ballSubTopic, queueSize, &Localizer::ballCallback, this);
  _ballPub = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(ballPubTopic, queueSize);
  for (auto& kv : pubTopics)
    _pubs[kv.first] = _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(kv.second, queueSize);
  _buffer.reserve(_bufferSize);
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

void Localizer::apriltagCallback(apriltag_ros::AprilTagDetectionArrayConstPtr msg) {
  // Collect transforms and current camera transform
  std::map<std::string, Eigen::Matrix4d> transforms;
  for (auto& detection : msg->detections) {
    std::string idStr = idsToString(detection.id);
    geometry_msgs::Quaternion rot = detection.pose.pose.pose.orientation;
    geometry_msgs::Point pos = detection.pose.pose.pose.position;
    Eigen::Quaterniond quat(rot.w, rot.x, rot.y, rot.z);
    Eigen::Vector3d vec(pos.x, pos.y, pos.z);

    if (idStr == _originId) {
      if (_buffer.size() == _bufferPos)
        _buffer.emplace_back();
      _buffer[_bufferPos] = std::make_pair(quat.toRotationMatrix(), vec);
      _bufferPos = (_bufferPos + 1) % _bufferSize;
    }
    if (_pubs.find(idStr) != _pubs.end()) {
      transforms[idStr] = combineMatrices(quat.toRotationMatrix(), vec);
    }
  }

  if (_buffer.size() == 0)
    return;

  // Naively average positions/rotations
  Eigen::Matrix3d avgRot = Eigen::Matrix3d::Zero();
  Eigen::Vector3d avgPos = Eigen::Vector3d::Zero();
  for (auto& item : _buffer) {
    avgRot += item.first;
    avgPos += item.second;
  }
  avgRot /= _buffer.size();
  avgPos /= _buffer.size();
  avgPos *= -1;

  // Kabsch algorithm
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(avgRot, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d ensureRHS = Eigen::Matrix3d::Identity();
  ensureRHS(2, 2) = ((svd.matrixV() * svd.matrixU().transpose()).determinant() > 0) ? 1.0 : -1.0;
  Eigen::Matrix3d camRotate = svd.matrixV() * ensureRHS * svd.matrixU().transpose();
  _transform = combineMatrices(camRotate, Eigen::Vector3d::Zero()) *
               combineMatrices(Eigen::Matrix3d::Identity(), avgPos);
  _pub.publish(toMsg(_transform, msg->header.stamp));

  // re-publish detections
  for (auto& kv : transforms) {
    auto& id = kv.first;
    auto& tagTransform = kv.second;
    _pubs[id].publish(toMsg(_transform * tagTransform, msg->header.stamp));
  }
}

void Localizer::ballCallback(geometry_msgs::Vector3StampedConstPtr msg) {
  Eigen::Vector4d camPos = _transform.col(3);
  if (camPos.z() == 0)
    return;

  /* saves the confidence of the ball position's validity */
  double confidence = msg->vector.z;


  Eigen::Vector4d camVec(msg->vector.x, msg->vector.y, 1, 0); // set tge z to 1 to restore the vector's properties
  Eigen::Vector4d vec = _transform * camVec;
  double t = (_ballRadius - camPos.z()) / vec.z();
  Eigen::Vector4d pos = camPos + t * vec;

  pos.z() = confidence;
  /* set the z of the new pose to the confidence of the ball position's validity */

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose.col(3) << pos;
  _ballPub.publish(toMsg(pose, msg->header.stamp));
}

Eigen::Matrix4d Localizer::combineMatrices(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.topLeftCorner<3, 3>() << rot;
  transform.col(3).head<3>() << pos;
  return transform;
}

geometry_msgs::PoseWithCovarianceStamped Localizer::toMsg(const Eigen::Matrix4d& transform,
                                                          ros::Time stamp,
                                                          const std::string& frameId) {
  Eigen::Quaterniond rot(transform.topLeftCorner<3, 3>());
  Eigen::Vector3d pos(transform.col(3).head<3>());

  geometry_msgs::PoseWithCovarianceStamped poseMsg;
  poseMsg.header.stamp = stamp;
  poseMsg.header.frame_id = frameId;
  poseMsg.pose.pose.position.x = pos.x();
  poseMsg.pose.pose.position.y = pos.y();
  poseMsg.pose.pose.position.z = pos.z();
  poseMsg.pose.pose.orientation.w = rot.w();
  poseMsg.pose.pose.orientation.x = rot.x();
  poseMsg.pose.pose.orientation.y = rot.y();
  poseMsg.pose.pose.orientation.z = rot.z();

  return poseMsg;
}

}  // namespace camera_tracking
