#ifndef CAMERA_TRACKING_TRANSFORM_REBROADCASTER_H
#define CAMERA_TRACKING_TRANSFORM_REBROADCASTER_H

#include <ros/ros.h>
#include <tf2_msgs/TFMessage.h>

namespace camera_tracking {

class TransformRebroadcaster {
public:
  TransformRebroadcaster(const ros::NodeHandle& nh);
  ~TransformRebroadcaster();

private:
  tf2_msgs::TFMessage tfAdapter(tf2_msgs::TFMessageConstPtr msg);
  void tfCallback(tf2_msgs::TFMessageConstPtr msg);
  void tfStaticCallback(tf2_msgs::TFMessageConstPtr msg);

  ros::NodeHandle _nh;
  unsigned int _queueSize;
  std::string _prefix;

  ros::Subscriber _tfSub;
  ros::Subscriber _tfStaticSub;
  ros::Publisher _tfPub;
  ros::Publisher _tfStaticPub;
};

}  // namespace camera_tracking

#endif  // CAMERA_TRACKING_TRANSFORM_REBROADCASTER_H