#include <ros/ros.h>
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

  ros::Timer timer =
      nh.createTimer(ros::Duration(1.0 / 20), [&](const ros::TimerEvent& e) {
        geometry_msgs::TransformStamped transform;
        try {
          transform = tfBuffer.lookupTransform(cameraName + "_" + originID,
                                               cameraName, ros::Time(0));
        } catch (tf2::TransformException e) {
          // ROS_WARN_THROTTLE(60, "%s", e.what());
          return;
        }
        transform.header.frame_id = "map";
        tfBroadcaster.sendTransform(transform);
      });

  ros::spin();
  return 0;
}
