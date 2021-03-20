#include "rocket_league_estimation/PHCModel.h"

//ros stuff
//#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//open cv stuff
#include <opencv2/opencv.hpp> //open cv core
#include <opencv2/highgui/highgui.hpp> //opencv window stuff
#include <opencv2/core/types.hpp> //convert pixel to irl cords
#include <cv_bridge/cv_bridge.h> //convert ros to open cv
#include <image_geometry/pinhole_camera_model.h>
//cpp includes
#include <iostream>
#include <string> 
#include <vector>


PHCModel::PHCModel() :

    model_nh{},
    cameraInfoSub{model_nh.subscribe(
        "cam0/camera_info", 1, &PHCModel::PinHoleCallback, this)}
    {    
       if (false) {
            throw std::runtime_error("Parameters not specified");
        }
    }
    void PHCModel::PinHoleCallback(const sensor_msgs::CameraInfo& info) {
        camera.fromCameraInfo(info);
    }