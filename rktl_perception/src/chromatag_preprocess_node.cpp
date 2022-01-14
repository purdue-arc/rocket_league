/* Node to convert colorspace for better tag detections.
 * License:
 *   BSD 3-Clause License
 *   Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
 *   All rights reserved.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "chromatag_preprocess");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("output", 1);
  image_transport::Subscriber sub = it.subscribe("input", 1,
    [&pub] (const sensor_msgs::ImageConstPtr& msg) {
      cv_bridge::CvImagePtr inImg = cv_bridge::toCvCopy(msg, "rgb8");   // convert ROS type to CV type
      cv:cvtColor(inImg->image, inImg->image, cv::COLOR_RGB2Lab);       // convert to LAB color space
      cv_bridge::CvImage outImg{inImg->header, "mono8"};                // create "grayscale" image to fool Apriltags
      cv::extractChannel(inImg->image, outImg.image, 1);                // extract A channel
      pub.publish(outImg.toImageMsg());                                 // publish
  });

  ros::spin();
  return 0;
}
