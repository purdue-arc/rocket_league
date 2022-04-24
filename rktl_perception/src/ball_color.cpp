#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>



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
#include "opencv2/highgui.hpp"
#include <opencv2/core/types.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h> 

/* messages */
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* math & vectors */
#include <math.h>
// #include <iostream>

using namespace cv;

const int max_value_H = 360 / 2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
int low_H = 0, low_S = 0, low_V = 0, dilate_amnt = 0, erode_amnt = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void*)
{
    low_H = min(high_H - 1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void*)
{
    high_H = max(high_H, low_H + 1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void*)
{
    low_S = min(high_S - 1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void*)
{
    high_S = max(high_S, low_S + 1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void*)
{
    low_V = min(high_V - 1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void*)
{
    high_V = max(high_V, low_V + 1);
    setTrackbarPos("High V", window_detection_name, high_V);
}
static void on_erode_thresh_trackbar(int, void*)
{
    erode_amnt = 0;
    setTrackbarPos("Erode", window_detection_name, erode_amnt);
}
static void on_dilate_thresh_trackbar(int, void*)
{
    dilate_amnt = 0;
    setTrackbarPos("Dilate", window_detection_name, dilate_amnt);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    try {
        /* Convert the ROS message into a cv_ptr & dereferencing the pointer to an OpenCV Mat */
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat current_frame = cv_ptr->image;
        cv::Mat frame_HSV, frame_threshold;
        // Convert from BGR to HSV colorspace
        cvtColor(current_frame, frame_HSV, COLOR_BGR2HSV);
        // Detect the object based on HSV Range Values
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
        dilate(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), dilate_amnt, 1, 1);
        erode(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), erode_amnt, 1, 1);

        // Show the frames
        imshow(window_detection_name, frame_threshold);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    createTrackbar("Erode", window_detection_name, &erode_amnt, 25, on_high_V_thresh_trackbar);
    createTrackbar("Dilate", window_detection_name, &dilate_amnt, 25, on_high_V_thresh_trackbar);
    Mat current_frame, frame_HSV, frame_threshold;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("image_rect_color", 10, imageCallback);
    ros::spin();
    cv::destroyWindow(window_detection_name);

    return 0;
}
/*
int main(int argc, char** argv) {
	cv::Mat im = imread("image.jpg", IMREAD_GRAYSCALE);
	if (im.empty())
	{
		std::cout << "Could not read the image" << std::endl;
		return 1;
	}

	
}*/