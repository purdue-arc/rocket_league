#include <camera_tracking/transform_rebroadcaster.h>

using namespace camera_tracking;

TransformRebroadcaster::TransformRebroadcaster(const ros::NodeHandle& nh,
                                               const ros::NodeHandle& pnh)
  : _nh(nh), _pnh(pnh), _queueSize(100) {
  // Get ROS Params
  ros::param::param<std::string>("~prefix", _prefix, "prefix");
  ROS_ERROR("prefix: %s", _prefix.c_str());

  // Set up Publishers
  _tfPub = _nh.advertise<tf2_msgs::TFMessage>("/tf", _queueSize);
  _tfStaticPub = _nh.advertise<tf2_msgs::TFMessage>("/tf_static", _queueSize);

  // Set up Subscribers
  _tfSub = _nh.subscribe("tf", _queueSize, &TransformRebroadcaster::tfCallback,
                         this);
  _tfStaticSub = _nh.subscribe("tf_static", _queueSize,
                               &TransformRebroadcaster::tfStaticCallback, this);
}

TransformRebroadcaster::~TransformRebroadcaster() {
}

tf2_msgs::TFMessage
TransformRebroadcaster::tfAdapter(tf2_msgs::TFMessageConstPtr msg) {
  tf2_msgs::TFMessage tfOut;
  tfOut.transforms.reserve(msg->transforms.size());

  for (const auto& inTransform : msg->transforms) {
    geometry_msgs::TransformStamped transform = inTransform;
    if (_prefix != transform.header.frame_id) {
      transform.header.frame_id = _prefix + "_" + transform.header.frame_id;
    }
    if (_prefix != transform.child_frame_id) {
      transform.child_frame_id = _prefix + "_" + transform.child_frame_id;
    }
    tfOut.transforms.push_back(transform);
  }

  return tfOut;
}

void TransformRebroadcaster::tfCallback(tf2_msgs::TFMessageConstPtr msg) {
  _tfPub.publish(tfAdapter(msg));
}

void TransformRebroadcaster::tfStaticCallback(tf2_msgs::TFMessageConstPtr msg) {
  _tfStaticPub.publish(tfAdapter(msg));
}