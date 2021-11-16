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

#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

class BallDetection {
public:
    BallDetection();
    ~BallDetection() = default;

private:
    void BallCallback(
        const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info);

    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    image_transport::ImageTransport image_transport;
    image_transport::CameraSubscriber camera_subscriber;
    ros::Publisher vecPub;
    ros::Subscriber detectionSub;
    int quad, height, minHue, minSat, minVib, maxHue, maxSat, maxVib;
    double originX, originY;
    bool showImage;
    image_geometry::PinholeCameraModel camera;

};