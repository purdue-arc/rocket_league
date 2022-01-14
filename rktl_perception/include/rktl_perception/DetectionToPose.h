/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

class DetectionToPose {
public:
    DetectionToPose();
    ~DetectionToPose() = default;

private:
    void DetectionCallback(
        const apriltag_ros::AprilTagDetectionArray& detection_msg);

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    ros::Publisher m_posePub;
    ros::Subscriber m_detectionSub;

    tf2_ros::Buffer m_tfBuffer;
    tf2_ros::TransformListener m_tfListener;
    tf2_ros::TransformBroadcaster m_tfBroadcaster;

    int m_parentTagId;
    std::string m_parentTagName;
    std::string m_parentBodyName;
    
    int m_childTagId;
    std::string m_childTagName;
    std::string m_childBodyName;

    const bool m_publishTf, m_publishPose;
};
