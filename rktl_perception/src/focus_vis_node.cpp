/* Node to help focus
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include "rktl_perception/focus_vis_node.h"

FocusNode::FocusNode() :
    nh{},
    pnh{"~"},
    image_transport{nh},
    imgPub{image_transport.advertise("edge_dectection", 1)},
    camera_subscriber{image_transport.subscribeCamera("image_rect_color", 10, &FocusNode::FocusCallback, this)}

    {}

void FocusNode::FocusCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info) {
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
        imgPub.publish(edgeImg);     
 
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("There was some error, likely with image conversion");
    }
}