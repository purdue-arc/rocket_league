# The `camera_tracking` package

This packages contains all nodes, launch files, and configuration files
associated with the
[`rocket_league`](https://github.com/purdue-arc/rocket_league/) project by
the [Purdue University Autonomous Robotics Club](https://www.purduearc.com/).

## Nodes
### `odom_gemerator`
Takes a rolling average of odom data to reduce noise.

Params:
- `buffer_size` (int): The number of datapoints to average. Default: `10`

### `ball_detection`
Uses color thresholding to find a vector pointing from the camera in the direction of the ball

Params:
- `publishThresh` (bool): Whether or not to publish the image after thresholding is applied. Default: `false`
- `minHue` (int): The lower value of the hue to threshold on a 0 - 255 scale. Default: `60`
- `minSat` (int): The lower value of the saturation to threshold on a 0 - 255 scale. Default: `135`
- `minVib` (int): The lower value of the vibrance to threshold on a 0 - 255 scale. Default: `50`
- `maxHue` (int): The upper value of the hue to threshold on a 0 - 255 scale. Default: `150`
- `maxSat` (int): The upper value of the saturation to threshold on a 0 - 255 scale. Default: `225`
- `maxVib` (int): The upper value of the vibrance to threshold on a 0 - 255 scale. Default: `255`

### `focus_vis`
Publishes a version of `image_color_rect` that has had Canny Edge Detection applied.

### `localizer`
Calculates the location of all objects in the cameras' view.

## Launch files
### `camera.launch`
This launch file launches all nodes and nodelets associcated with a single
camera. Most arguments can be safely ignored, with the exception of
`camera_name`

To run: `roslaunch camera_tracking camera.launch`

Arguments:
- `camera_name` (string): The name of the camera to be launched. All topics
  will be published under this name, and configuration details will be loaded
  from the `.yaml` with the same name from the `config` directory. See the
  'Configuration files' section for more information. Default: `aravis_cam`.
- `load_manager` (boolean): Whether or not a nodelet manager should be created
  Only one nodelet manager should exist for all cameras. Default: `true`.
- `manager_name` (string): The name of the nodelet manager. If `load_manager` is
  `true`, a nodelet manager is created with this name. If `load_manager`
  is `false`, the nodelet manager with this name is used. Default:
  `camera_manager`.
- `manager_threads` (integer): The number of threads used by the nodelet
  manager, if created. Default: `4`.
- `launch_dynamic_reconfigure` (boolean): Whether or not `rqt_reconfigure`
  should be launched (One instance of `rqt_reconfigure` is enough to
  control all nodes that use it, including all cameras). Default: value of
  `load_manager`.

### `focus_assist.launch`
This launch file launches the four nodes that generate a Canny Edge Detection copy of every camera, as to aide in focusing the cameras properly.

To run: `roslaunch camera_tracking focus_assist.launch`


## Configuration files
Configuration files are located in the `config` directory. They contain configuration information for different cameras, inculding sensor information used by the GenICam standard and distortion/rectification/rectifcation matrices.

Camera Calibration Info. Gotten from running
`rosrun camera_calibration cameracalibrator.py` and copy/pasting the
contents of the resulting `.yml` file into this config file. See the
[`camera_calibration`](https://wiki.ros.org/camera_calibration) package for
more info.