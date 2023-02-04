/* Node to detect the ball's position on the field.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#ifndef __RKTL_PERCEPTION_BALL_DETECTION_H__
#define __RKTL_PERCEPTION_BALL_DETECTION_H__

/* ros */
#include <ros/ros.h>
#include <ros/console.h>

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
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* math & vectors */
#include <vector>
#include <math.h>

namespace rktl_perception {

class BallDetection {
public:
    BallDetection(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
    ~BallDetection() = default;

private:
    void ballCallback(const sensor_msgs::ImageConstPtr& msg,
                      const sensor_msgs::CameraInfoConstPtr& info);

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    image_transport::ImageTransport _it;
    image_transport::CameraSubscriber _cameraSub;
    image_transport::Publisher _imgPub;
    ros::Publisher _vecPub;
    ros::Subscriber _detectionSub;
    int _minHue, _minSat, _minVib, _maxHue, _maxSat, _maxVib, _minSize, _erodeAmnt, _dilateAmnt;
    double _originX, _originY;
    bool _publishThresh;
    image_geometry::PinholeCameraModel _camera;
};

}  // namespace rktl_perception

#endif  // __RKTL_PERCEPTION_BALL_DETECTION_H__
