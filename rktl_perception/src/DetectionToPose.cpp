/* TODO
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include "rktl_perception/DetectionToPose.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DetectionToPose::DetectionToPose() :
    m_nh{},
    m_pnh{"~"},
    m_posePub{m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "pose", 1)},
    m_detectionSub{m_nh.subscribe(
        "tag_detections", 1, &DetectionToPose::DetectionCallback, this)},
    m_tfBuffer{},
    m_tfListener{m_tfBuffer},
    m_tfBroadcaster{},
    m_parentTagId{},
    m_parentTagName{},
    m_parentBodyName{},
    m_childTagId{},
    m_childTagName{},
    m_childBodyName{},
    m_publishTf{m_pnh.param<bool>("publish/tf", false)},
    m_publishPose{m_pnh.param<bool>("publish/pose", true)}
    {
        if (!m_pnh.getParam("parent/tag_id", m_parentTagId) ||
                !m_pnh.getParam("parent/tag_name", m_parentTagName) ||
                !m_pnh.getParam("parent/body_name", m_parentBodyName) ||
                !m_pnh.getParam("child/tag_id", m_childTagId) ||
                !m_pnh.getParam("child/tag_name", m_childTagName) ||
                !m_pnh.getParam("child/body_name", m_childBodyName)) {
            throw std::runtime_error("Parameters not specified");
        }
    }

void DetectionToPose::DetectionCallback(
        const apriltag_ros::AprilTagDetectionArray& detection_msg) {
    // pull transform between id's from message

    tf2::Transform parent_tag_to_cam{tf2::Transform::getIdentity()};
    tf2::Transform child_tag_to_cam{tf2::Transform::getIdentity()};
    for (auto it = detection_msg.detections.begin();
            it != detection_msg.detections.end(); ++it) {
        if (it->id[0] == m_parentTagId) {
            tf2::fromMsg(it->pose.pose.pose, parent_tag_to_cam);
        }
        if (it->id[0] == m_childTagId) {
            tf2::fromMsg(it->pose.pose.pose, child_tag_to_cam);
        }
    }
    if (parent_tag_to_cam == tf2::Transform::getIdentity() ||
            child_tag_to_cam == tf2::Transform::getIdentity()) {
        ROS_WARN_THROTTLE(10, "Could not find all detections.");
        return;
    }

    // pull the transforms between frame and tags from tf tree

    if (!m_tfBuffer.canTransform(m_parentBodyName, m_parentTagName, ros::Time(0)) ||
            !m_tfBuffer.canTransform(m_childBodyName, m_childTagName, ros::Time(0))) {
        ROS_WARN_THROTTLE(10, "Could not find all transformations.");
        return;
    }
    tf2::Transform parent_body_to_tag, child_body_to_tag;
    tf2::fromMsg(m_tfBuffer.lookupTransform(m_parentBodyName, m_parentTagName,
        ros::Time(0)).transform, parent_body_to_tag);
    tf2::fromMsg(m_tfBuffer.lookupTransform(m_childBodyName, m_childTagName, 
        ros::Time(0)).transform, child_body_to_tag);

    // construct final transformation

    tf2::Transform child_body_to_parent = parent_body_to_tag.inverse() *
        parent_tag_to_cam.inverse() * child_tag_to_cam * child_body_to_tag;

    // send to tf tree

    if (m_publishTf) {
        geometry_msgs::TransformStamped tfs;
        tfs.header.stamp = detection_msg.header.stamp;
        tfs.header.frame_id = m_parentBodyName;
        tfs.child_frame_id = m_childBodyName;
        tfs.transform = tf2::toMsg(child_body_to_parent);
        m_tfBroadcaster.sendTransform(tfs);
    }

    // publish as ROS message

    if (m_publishPose) {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = detection_msg.header.stamp;
        msg.header.frame_id = m_parentBodyName;
        tf2::toMsg(child_body_to_parent, msg.pose.pose);
        // TODO: handle covariances
        m_posePub.publish(msg);
    }
}
