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

#include "rocket_league_estimation/BallDetection.h"

//ros stuff
//#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//open cv stuff
#include <opencv2/opencv.hpp> //open cv core
#include <opencv2/highgui/highgui.hpp> //opencv window stuff
#include <cv_bridge/cv_bridge.h> //convert ros to open cv

//cpp includes
#include <iostream>
#include <string> 
#include <vector>

int getMaxAreaContourId(std::vector <std::vector<cv::Point>> contours);

BallDetection::BallDetection() :
    nh{},
    pnh{"~"},
    posePub{nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "ball_pose", 1)},
    detectionSub{nh.subscribe(
        "image_rect_color", 1, &BallDetection::BallCallback, this)},
    minHue{pnh.param<int>("min_hue", 60)},
    minSat{pnh.param<int>("min_sat", 135)},
    minVib{pnh.param<int>("min_vib", 50)},
    maxHue{pnh.param<int>("max_hue", 150)},
    maxSat{pnh.param<int>("max_sat", 255)},
    maxVib{pnh.param<int>("max_vib", 255)}
    {    
       if (false) {
            throw std::runtime_error("Parameters not specified");
            }
    }

void BallDetection::BallCallback(
        const sensor_msgs::ImageConstPtr& msg) {
            std::cout << maxVib;
            cv_bridge::CvImagePtr cv_ptr;

            try {
                // Convert the ROS message  
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                // Store the values of the OpenCV-compatible image
                // into the current_frame variable
                cv::Mat current_frame = cv_ptr->image;
                //cv::resize(current_frame, current_frame, cv::Size(), 0.25, 0.25);
                cv::Mat frame_HSV, frame_threshold;
                // Convert from BGR to HSV colorspace
                cvtColor(current_frame, frame_HSV, cv::COLOR_BGR2HSV);
                // Detect the object based on HSV Range Values
                inRange(frame_HSV, cv::Scalar(minHue, minSat, minVib), cv::Scalar(maxHue, maxSat, maxVib), frame_threshold);
                erode(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
                dilate(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
                //find the centers
                std::vector<std::vector<cv::Point> > contours;
                std::vector<cv::Vec4i> hierarchy;
                findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
                //find largest contour
                cv::Moments moment = cv::moments(contours.at(getMaxAreaContourId(contours)));
                //calculates the center
                double centerX = moment.m10 / moment.m00;
                double centerY = moment.m01 / moment.m00;
                //publishing This is the part that i get super unsure of.
                geometry_msgs::PoseWithCovarianceStamped pose;
                pose.header = msg->header;
                // set x,y coord
                pose.pose.pose.position.x = centerX;
                pose.pose.pose.position.y = centerY;
                pose.pose.pose.position.z = 0.0;
                pose.pose.pose.orientation.x = 0.0;
                pose.pose.pose.orientation.y = 0.0;
                pose.pose.pose.orientation.z = 0.0;
                pose.pose.pose.orientation.w = 1.0;
                ROS_INFO("x: %f, y: %f, z: 0.0", centerX, centerY);
                posePub.publish(pose);
                // Display frame for 30 milliseconds
                //cv::imshow("view", frame_threshold);
                //cv::waitKey(30);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
            }
    // publish as ROS message
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
