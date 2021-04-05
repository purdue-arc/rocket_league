#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "camera_origin");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  std::string topicName;
  std::string targetFrame;
  std::string sourceFrame;
  double updateRate;

  if (!pnh.getParam("topic_name", topicName)) {
    ROS_ERROR("Required Param \"topic_name\" not set. Exiting.");
    return -1;
  }
  if (!pnh.getParam("target_frame", targetFrame)) {
    ROS_ERROR("Required Param \"target_frame\" not set. Exiting.");
    return -1;
  }
  if (!pnh.getParam("source_frame", sourceFrame)) {
    ROS_ERROR("Required Param \"source_frame\" not set. Exiting.");
    return -1;
  }
  pnh.param<double>("update_rate", updateRate, 0.1);

  ros::Publisher pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(topicName, 1);
  geometry_msgs::PoseWithCovarianceStamped identity;

  ros::Timer timer = nh.createTimer(
      ros::Duration(updateRate), [&](const ros::TimerEvent& e) -> void {
        geometry_msgs::TransformStamped transform;
        try {
          transform =
              tfBuffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
          geometry_msgs::PoseWithCovarianceStamped pose;
          tf2::doTransform(identity, pose, transform);
          pose.header = transform.header;
          pub.publish(pose);
        } catch (tf2::TransformException e) {
          return;
        }
      });

  ros::spin();
  return 0;
}
