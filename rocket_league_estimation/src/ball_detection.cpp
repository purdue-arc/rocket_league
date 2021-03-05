//ros stuff
#include <ros/ros.h>
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


int main(int argc, char** argv)
{
    // The name of the node
    ros::init(argc, argv, "ball_detection");

    // Default handler for nodes in ROS
    ros::NodeHandle nh;

    // Used to subscribe to images
    image_transport::ImageTransport it(nh);

    // Used to publish Pose
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("ball_pose", 1);//not sure abt this


    // Subscribe to the /camera topic
    image_transport::Subscriber sub = it.subscribe("aravis_cam/image_rect_color", 1, imageCallback);//not sure abt this

    // Make sure we keep reading new video frames by calling the imageCallback function
    ros::spin();

    // Close down OpenCV
    //cv::destroyWindow("view");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    // Pointer used for the conversion from a ROS message to 
    // an OpenCV-compatible image
    cv_bridge::CvImagePtr cv_ptr;

    try
    {

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
        inRange(frame_HSV, cv::Scalar(60, 135, 50), cv::Scalar(110, 255, 255), frame_threshold);
        erode(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        dilate(frame_threshold, frame_threshold, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

        //find the centers
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        findContours(frame_threshold, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        //find largest contour
        cv::Moments moment = moments(contours.at(getMaxAreaContourId(contours)));
        //calculates the center
        double centerX = moment.m10 / moment.m00;
        double centerY = moment.m01 / moment.m00;

        //publishing This is the part that i get super unsure of.
        geometry_msgs::PoseWithCovarianceStamped pose;
        pose.header.frame_id = fixed_frame;
        pose.header.stamp = ros::Time::now();

        // set x,y coord
        pose.pose.pose.position.x = centerX;
        pose.pose.pose.position.y = centerY;
        pose.pose.pose.position.z = 0.0;

        pose.pose.pose.orentation.x = 0.0;
        pose.pose.pose.orentation.y = 0.0;
        pose.pose.pose.orentation.z = 0.0;
        pose.pose.pose.orentation.w = 1.0;


        ROS_INFO("x: %f, y: %f, z: 0.0", centerX, centerY);
        pub_.publish(pose);

        // Display frame for 30 milliseconds
        //cv::imshow("view", frame_threshold);
        //cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
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
