#include <camera_tracking/transform_rebroadcaster.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "tf_broadcast");
  ros::NodeHandle nh;

  camera_tracking::TransformRebroadcaster rebroadcaster(nh);

  ros::spin();
  return 0;
}
