/*******************************************************************************
* BSD 3-Clause License
* Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "camera_tracking/ball_detection_node.h"
//#include "rocket_league_estimation/PHCModel.h"


//ros stuff
//#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <image_transport/camera_subscriber.h>

//open cv stuff
#include <opencv2/opencv.hpp> //open cv core
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> //opencv window stuff
#include <opencv2/core/types.hpp> //convert pixel to irl cords
#include <cv_bridge/cv_bridge.h> //convert ros to open cv
#include <image_geometry/pinhole_camera_model.h>
//cpp includes
#include <iostream>
#include <string> 
#include <vector>
#include <math.h>

int getMaxAreaContourId(std::vector <std::vector<cv::Point>> contours);

BallDetection::BallDetection() :
    nh{},
    pnh{"~"},
    image_transport{nh},
    vecPub{nh.advertise<geometry_msgs::Vector3Stamped>("ball_vec", 10)},
    imgPub{image_transport.advertise("threshold_img", 1)},
    camera_subscriber{image_transport.subscribeCamera("image_rect_color", 10, &BallDetection::BallCallback, this)},

    /*
     * future plan for this node is to make all of these adjustable with dynamic_reconfigure
     */

    showImage{pnh.param<bool>("showImage", false)},
    minHue{pnh.param<int>("min_hue", 060)},
    minSat{pnh.param<int>("min_sat", 135)},
    minVib{pnh.param<int>("min_vib", 050)},
    maxHue{pnh.param<int>("max_hue", 150)},
    maxSat{pnh.param<int>("max_sat", 255)},
    maxVib{pnh.param<int>("max_vib", 255)}
    {}

void BallDetection::BallCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        //define published messages
        geometry_msgs::Vector3Stamped vec;
        sensor_msgs::Image threshImg;
        // Convert the ROS message  
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        // Store the values of the OpenCV-compatible image
        // into the current_frame variable
        cv::Mat current_frame = cv_ptr->image;
        cv::Mat frame_HSV, frame_threshold;
        //resize image
        cv::resize(current_frame, current_frame, cv::Size(), 0.5, 0.5);
        // Convert from BGR to HSV colorspace
        cvtColor(current_frame, frame_HSV, cv::COLOR_BGR2HSV);
        //get image size
        int h = current_frame.size().height;
        int w = current_frame.size().width;
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, cv::Scalar(minHue, minSat, minVib), cv::Scalar(maxHue, maxSat, maxVib), frame_threshold);
        erode(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        dilate(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        //find the centers
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        if (contours.size() != 0) {
            //find largest contour
            cv::Moments moment = cv::moments(contours.at(getMaxAreaContourId(contours)));
            //calculates the center
            double centerX = moment.m10 / moment.m00;
            double centerY = moment.m01 / moment.m00;

            double centerdX = (w/2) - centerX;
            double centerdY = (h/2) - centerY;

            //create camera model
            camera.fromCameraInfo(info);
            cv::Point3d cam = camera.projectPixelTo3dRay(cv::Point2d(centerX, centerY));
            
            //create the vector to publish
            vec.vector.x = cam.x;
            vec.vector.y = cam.y;
            vec.vector.z = cam.z;
            
            if (showImage) {
                
                /* publishes the threshold image */

                std_msgs::Header header;
                cv_bridge::CvImage img_bridge;
                sensor_msgs::Image img_msg;
                header.stamp = ros::Time::now(); 
                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame_threshold);
                img_bridge.toImageMsg(img_msg);
                imgPub.publish(img_msg);     
            }
            
            vec.header = msg->header;
            vecPub.publish(vec);
        }
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int getMaxAreaContourId(std::vector <std::vector<cv::Point>> contours) {
    double maxArea = 0;
    //calculates the area of each contour and then returns the largest one.
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        }
    }
    return maxAreaContourId;
}