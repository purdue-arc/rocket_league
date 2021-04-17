#include <ros/ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "camera_origin");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  std::string cameraName;
  std::string originID;

  ros::param::param<std::string>("~camera_name", cameraName, "camera");
  ros::param::param<std::string>("~origin_id", originID, "origin");

  std::string targetFrame = cameraName + "_" + originID;
  std::string sourceFrame = cameraName;

  while (ros::ok()) {
    try {
      geometry_msgs::TransformStamped transform;
      transform =
          tfBuffer.lookupTransform(cameraName + "_" + originID, cameraName,
                                   ros::Time::now(), ros::Duration(0.1));
      transform.header.frame_id = "map";
      tfBroadcaster.sendTransform(transform);
    } catch (tf2::TransformException e) {
      // ROS_WARN("Cannot do transformation: %s", e.what());
    }
    ros::spinOnce();
  }
  return 0;
}
