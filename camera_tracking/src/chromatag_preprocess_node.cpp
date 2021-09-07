#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

std::unique_ptr<image_transport::Publisher> pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr inImg = cv_bridge::toCvCopy(msg, "bgr8");   // convert ROS type to CV type
  cv:cvCvtColor(&inImg->image, &inImg->image, cv::COLOR_BGR2Lab);   // convert to LAB color space
  cv_bridge::CvImage outImg{inImg->header, "mono8"};                // create "grayscale" image to fool Apriltags
  cv::extractChannel(inImg->image, outImg.image, 1);                // extract A channel
  pub->publish(outImg.toImageMsg());                                // publish
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "chromatag_preprocess");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("input", 1, imageCallback);
  pub = std::make_unique<image_transport::Publisher>(it.advertise("output", 1));

  ros::spin();
  return 0;
}
