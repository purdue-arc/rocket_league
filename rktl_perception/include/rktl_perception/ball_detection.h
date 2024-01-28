/* Node to detect the ball's position on the field.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#ifndef __RKTL_PERCEPTION_BALL_DETECTION_H__
#define __RKTL_PERCEPTION_BALL_DETECTION_H__

/* ros */
#include <rclcpp/rclcpp.hpp>


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
#include <memory>

namespace rktl_perception {

class BallDetection {
public:
    BallDetection(const std::shared_ptr<rclcpp::Node>& node);
    ~BallDetection() = default;

private:
    void ballCallback(const sensor_msgs::ImageConstPtr& msg,
                      const sensor_msgs::CameraInfoConstPtr& info);

    std::shared_ptr<rclcpp::Node> _node;
    image_transport::ImageTransport _it;
    image_transport::CameraSubscriber _cameraSub;
    image_transport::Publisher _imgPub;
    rclcpp::Publisher _vecPub;
    rclcpp::Subscribtion _detectionSub;
    int _minHue, _minSat, _minVib, _maxHue, _maxSat, _maxVib, _minSize, _erodeAmnt, _dilateAmnt;
    double _originX, _originY;
    bool _publishThresh;
    image_geometry::PinholeCameraModel _camera;
};

}  // namespace rktl_perception

#endif  // __RKTL_PERCEPTION_BALL_DETECTION_H__
