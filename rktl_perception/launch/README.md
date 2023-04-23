# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_perception <launch file>
```

```{contents} Launch Files in this package
:depth: 2
:backlinks: top
:local: true
```

---

## cal.launch

Utility launch file that launches an interface for calibrating all cameras. It
is configured for a 9 squares by 7 squares chessboard pattern, where each square
is 26cm is length.

---

## camera.launch

Launch file containing everything that needs to be run for 1 camera. The
`camera_name` argument can be changed to easily change the camera used.

### Launch Arguments

- `load_manager` (default: true): If true, a nodelet manager is started. If a
    nodelet manager is already running, set this to false to avoid any errors.
- `manager_name` (default: `camera_manager`): The name of the nodelet manager to
    use for nodes that require it.
- `manager_threads` (default: `4`): Number of threads to use in the nodelet
    manager, if started.
- `camera_name` (default: `cam0`): Name of the camera to launch the stack for.

### Nodes

- `cams/{camera_name}/{manager_name}`: [Nodelet](https://wiki.ros.org/nodelet)
    manager. Only run if the `load_manager` argument is set to true.
- `cams/{camera_name}/{camera_name}`: [Nodelet](https://wiki.ros.org/nodelet) of type
    [pointgrey_camera_driver/PointGreyCameraNodelet](https://wiki.ros.org/pointgrey_camera_driver).
    Puts camera feed onto the ROS network. Loads parameters (including camera
    serial number) from `rktl_perception/config/{camera_name}/pointgrey.yaml`
    and calibration from `rktl_perception/config/{camera_name}/calibration.yaml`.
- `cams/{camera_name}/apriltag`: `apriltag_ros_continuous_node` node from the
    [apriltag_ros](https://wiki.ros.org/apriltag_ros) package. Looks for
    AprilTags in the camera feed and outputs the result to be used by other
    nodes. Loads parameters from `rktl_perception/config/apriltag_settings.yaml`
    and `rktl_perception/config/tags.yaml`.
- `localizer`: [`localizer`](../nodes/README.md#localizer) node from the
    `rktl_perception` package.
- `pose_to_tf`: [`pose_to_tf`](../nodes/README.md#pose_to_tf) node from the
    `rktl_perception` package.
- `ball_detection`: [`ball_detection`](../nodes/README.md#ball_detection) node
    from the `rktl_perception` package. Loads parameters from
    `rktl_perception/config/ball_settings.yaml`.

### Included Launch Files

- `image_proc image_proc.launch`: Basic processing stack (image rectification
    for the provided camera).

---

## color_picker.launch

Utility launch file that opens a color picker used for fine tuning the parameters used for ball detection.

---

## field_view.launch

Utility launch file used for displaying the camera positions and feeds in RViz.

---

## focus_assist.launch

Utility launch file used launching the topics needed for focusing camera lenses.

---
