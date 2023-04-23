# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```
rosrun rktl_perception <Node name>
```

:::{contents} ROS Nodes in this package
:depth: 2
:backlinks: top
:local: true
:::

---

## ball_detection

C++ Node. Uses color thresholding to find a vector pointing from the camera in
the direction of the ball.

### Subscribed Topics

- `image_rect_color` ([image_transport Camera](https://wiki.ros.org/image_transport)):
    Full-color camera feed.

### Published Topics

- `ball_vec` ([geometry_msgs/Vector3Stamped](/geometry_msgs/html/msg/Vector3Stamped.html#http://)):
    The $x$ and $y$ components of the vector comprise the the unit vector which
    passes from the camera center to the detected ball. The $z$ component is
    the largest contour area detected (the size of the ball as it appears to
    the camera).
- `threshold_img` ([image_transport Camera](https://wiki.ros.org/image_transport)):
    The black-and-white output of the thresholding. Only published to if the
    `publishThresh` parameter is set to true.

### Parameters

- `publishThresh` (bool, default: false): Whether or not to publish the image
    after thresholding is applied.
- `minHue` (int, default: `60`): The lower value of the hue to threshold on a
    0 - 255 scale.
- `minSat` (int, default: `135`): The lower value of the saturation to
    threshold on a 0 - 255 scale. 
- `minVib` (int, default: `50`): The lower value of the vibrance to threshold
    on a 0 - 255 scale.
- `maxHue` (int, default: `150`): The upper value of the hue to threshold on a
    0 - 255 scale.
- `maxSat` (int, default: `225`): The upper value of the saturation to
    threshold on a 0 - 255 scale.
- `maxVib` (int, default: `255`): The upper value of the vibrance to threshold
    on a 0 - 255 scale.

---

## focus_vis

C++ Node. Publishes a version of `image_color_rect` that has had Canny Edge
Detection applied. Useful when using a
[siemens star](https://en.wikipedia.org/wiki/Siemens_star) to get the focus of
the camera just right.

### Subscribed Topics

- `image_rect_color` ([image_transport Camera](https://wiki.ros.org/image_transport)):
    Full-color camera feed.

### Published Topics

- `edge_dectection` ([image_transport Camera](https://wiki.ros.org/image_transport)):
    The black-and-white output of the edge detection.

---

## localizer

C++ Node. Calculates the location of all objects in the cameras' view.
Republishes AprilTag and ball detections (received in "camera space") into a
common reference frame for all cameras (referred to as "world space"). The
measurements used to put the camera into world space are "averaged out" over a
series of measurements to reduce noise.

Multiple topics are published to, given by the `pub_topics` parameter. This
parameter is interpreted as key/value pairs, where the key is an
[AprilTag](https://wiki.ros.org/apriltag_ros) ID or the IDs of an AprilTag
bundle and the value is the name of the topic that should be published to. In
the case of a bundle, the tag IDs are given in ascending order, delimited by
commas (e.g. `"0,1,2,3"`).

For the inclined, "camera space" is defined as the space whose origin is defined
as the center of the camera, where the $x$-axis points to the right of the
image, the $y$-axis points towards the top of the image, and the camera looks
down the positive $z$-axis (this is the same convention that OpenCV uses).
"World space" is defined as the space whose origin is the center of the field,
where the $x$-axis points towards the "right-hand" goals, the $y$-axis points
towards the "top" sideline, and the $z$-axis points directly up.

### Subscribed Topics

- `tag_detections` ([apriltag_ros/AprilTagDetectionArray](/apriltag_ros/html/msg/AprilTagDetectionArray.html#http://)):
    Output of an AprilTag detection node. The name of this topic can be easily
    adjusted by modifying the `dectection_topic` parameter.
- `ball_vector` ([geometry_msgs/Vector3Stamped](/geometry_msgs/html/msg/Vector3Stamped.html#http://)):
    Output of the [`ball_detection`](#ball-detection) node. The name of this
    topic can be easily adjusted by modifying the `ball_sub_topic` parameter.

### Published Topics

- `{pub_topic}` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    The position/orientation of the camera in world space.
- `{pub_topics.values}` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    The position/orientation of the given objects in world space. `pub_topics`
    is a dictionary (`key`/`value` pairs) where each detections of AprilTag
    with ID `key` is published on the topic named `value`.

### Parameters

- `dectection_topic` (string, default: `"tag_detections"`): Name of topic
    subscribed to, published to by an AprilTag detection node.
- `origin_id` (string, default: `"0"`): ID of the AprilTag or AprilTag bundle
    that defines the origin of world space, i.e. the center of the field.
- `pub_topic` (string) Topic name to publish camera pose to.
- `pub_topics` (dictionary): `key`/`value` pairs where each detections of
    AprilTag with ID `key` is published on the topic named `value`.
- `ball_sub_topic` (string, default: `"ball_vector"`): Name of topic
    subscribed to, published to by a [`ball_detection`](#ball-detection) node.
- `ball_pub_topic` (string, default: `"ball_pos"`): Name of topic
    published containing the position of the ball in world space.
- `ball_radius` (float, default: `0.05`): Radius of the ball, in meters.
- `buffer_size` (int, default: `10`): Number of elements used in the "averaging"
    of the camera pose.
- `queue_size` (int, default: `100`): Size of the the queue for the publishers.

---

## pose_to_tf

Publishes an output of the [localizer](#localizer) node onto the [tf tree](https://wiki.ros.org/tf2).

### Subscribed Topics

- `pose` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    Data to republish onto the TF tree.

### Parameters

- `~cam_frame_id` (string, default: `"cam0"`): Frame ID of the camera.

---

## projector

Create a depth map to project camera data onto ground plane.

### Subscribed Topics

- `projector` ([sensor_msgs/CameraInfo](/sensor_msgs/html/msg/CameraInfo.html#http://)):
    Camera Info, containing the projection matrix used to re-project the image
    onto the plane.

### Published Topics

- `depth_rect` ([sensor_msgs/Image](/sensor_msgs/html/msg/Image.html#http://)):
    Calculated depth of each pixel in the camera.

### Parameters

- `~ground_height` (float, default: `0.0`): Height of the ground, in meters.
- `~update_period` (float, default: `1`): Rate at which to re-compute depth map,
    in updates per second.

---
