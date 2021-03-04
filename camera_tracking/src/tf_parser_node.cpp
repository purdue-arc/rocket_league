#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tf_parser");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  double updateRate;
  std::vector<std::string> cameraNames;
  std::vector<std::string> tagNames;

  ros::param::param<double>("~update_rate", updateRate, 0.1);
  ros::param::get("~camera_names", cameraNames);
  ros::param::get("~tag_names", tagNames);

  for (auto& name : cameraNames)
    ROS_ERROR("camera name: %s", name.c_str());

  ros::Timer timer = nh.createTimer(
      ros::Duration(updateRate), [&](const ros::TimerEvent& e) -> void {
        geometry_msgs::TransformStamped transform;
        for (auto& tagName : tagNames) {
          geometry_msgs::TransformStamped tagTransform;
          tagTransform.header.stamp = ros::Time(0);
          for (auto& cameraName : cameraNames) {
            try {
              transform = tfBuffer.lookupTransform(
                  "map", cameraName + "_" + tagName, ros::Time(0));
              if (transform.header.stamp > tagTransform.header.stamp) {
                tagTransform = transform;
                tagTransform.child_frame_id = tagName;
              }
            } catch (tf2::TransformException e) {
              // Frame doesn't exist
              continue;
            }
          }

          if (tagTransform.header.stamp != ros::Time(0)) {
            tfBroadcaster.sendTransform(tagTransform);
          }
        }
      });

  ros::spin();
  return 0;
}
