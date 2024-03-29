/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#ifndef __RKTL_PERCEPTION_LOCALIZER_H__
#define __RKTL_PERCEPTION_LOCALIZER_H__

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Dense>

namespace rktl_perception {

class Localizer {
public:
    Localizer(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~Localizer() = default;

    static std::string idsToString(std::vector<int> ids);

private:
    void apriltagCallback(apriltag_ros::AprilTagDetectionArrayConstPtr msg);
    void ballCallback(geometry_msgs::Vector3StampedConstPtr msg);
    Eigen::Matrix4d combineMatrices(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos);
    geometry_msgs::PoseWithCovarianceStamped toMsg(const Eigen::Matrix4d& transform,
                                                   ros::Time stamp,
                                                   const std::string& frameId = "map");

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Subscriber _sub;
    std::string _originId;
    ros::Publisher _pub;
    std::map<std::string, ros::Publisher> _pubs;
    ros::Subscriber _ballSub;
    ros::Publisher _ballPub;
    int _bufferSize;
    int _bufferPos;
    double _ballRadius;
    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> _buffer;
    Eigen::Matrix4d _transform;
};

}  // namespace rktl_perception

#endif  // __RKTL_PERCEPTION_LOCALIZER_H__
