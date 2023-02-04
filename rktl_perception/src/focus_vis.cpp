/* Node to help focus
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <rktl_perception/focus_vis.h>

namespace rktl_perception {

FocusVis::FocusVis(const ros::NodeHandle& nh) : _nh(nh), _it(_nh) {
    _imgPub = _it.advertise("edge_dectection", 1);
    _cameraSub = _it.subscribeCamera("image_rect_color", 10, &FocusVis::focusCallback, this);
}

void FocusVis::focusCallback(const sensor_msgs::ImageConstPtr& msg,
                             const sensor_msgs::CameraInfoConstPtr& info) {
    try {
        /* Convert the ROS message into a cv_ptr & dereferencing the pointer to an OpenCV Mat */
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat current_frame = cv_ptr->image;
        cv::Mat frame_gray, frame_blurred, frame_edge;

        /* Convert from BGR to grey scale colorspace */
        cvtColor(current_frame, frame_gray, cv::COLOR_BGR2GRAY);

        cv::GaussianBlur(frame_gray, frame_blurred, cv::Size(5, 5), 1.5);

        cv::Canny(frame_blurred, frame_edge, 100, 200);
        /* publishes the threshold image */

        sensor_msgs::Image edgeImg;
        std_msgs::Header header;
        cv_bridge::CvImage img_bridge;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, frame_edge);
        img_bridge.toImageMsg(edgeImg);
        _imgPub.publish(edgeImg);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("There was some error, likely with image conversion");
    }
}

}  // namespace rktl_perception
