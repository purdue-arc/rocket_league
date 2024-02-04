/* Node to help focus
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#ifndef __RKTL_PERCEPTION_FOCUS_VIS_H__
#define __RKTL_PERCEPTION_FOCUS_VIS_H__

/* ros */
#include <ros/ros.h>

/* image */
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <image_geometry/pinhole_camera_model.h>

/* opencv */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>

/* messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* math & vectors */
#include <vector>
#include <math.h>

namespace rktl_perception {

class FocusVis {
public:
    FocusVis(const ros::NodeHandle& nh);
    ~FocusVis() = default;

private:
    void focusCallback(const sensor_msgs::ImageConstPtr& msg,
                       const sensor_msgs::CameraInfoConstPtr& info);

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    image_transport::ImageTransport _it;
    image_transport::CameraSubscriber _cameraSub;
    image_transport::Publisher _imgPub;
    ros::Subscriber _detectionSub;
    image_geometry::PinholeCameraModel _camera;
};

}  // namespace rktl_perception

#endif  // __CAMERA_TRACKING_FOCUS_VIS_H__
