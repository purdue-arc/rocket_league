#include <camera_tracking/transform_rebroadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tf_broadcast_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  camera_tracking::TransformRebroadcaster rebroadcaster(nh, pnh);

  ros::spin();
  return 0;
}
