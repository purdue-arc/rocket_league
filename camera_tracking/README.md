# The ```camera_tracking``` package

This packages contains all nodes, launch files, and configuration files
associated with the
[```rocket_league```](https://github.com/purdue-arc/rocket_league/) project by
the [Purdue University Autonomous Robotics Club](https://www.purduearc.com/).
All development is done on the
[```camera-dev```](https://github.com/purdue-arc/rocket_league/tree/camera-dev)
branch.

## Dependencies
- [```camera_aravis```](https://github.com/purdue-arc/camera_aravis)
- [```image_proc```](https://wiki.ros.org/image_proc)

## Nodes
None currently

## Launch files
### ```single_camera.launch```
This launch file launches all nodes and nodelets associcated with a single
camera. Most arguments can be safely ignored, with the exception of
```camera_name```

To run: ```roslaunch camera_tracking single_camera.launch```

Arguments:
- ```camera_name``` (string): The name of the camera to be launched. All topics will be
  published under this name, and configuration details will be loaded from
  the ```.yaml``` with the same name from the ```config``` directory. See the
  'Configuration files' section for more information. Default: ```aravis_cam```.
- ```load_manager``` (boolean): Whether or not a nodelet manager should be created. Only
  one nodelet manager should exist for all cameras. Default: ```true```.
- ```manager_name``` (string): The name of the nodelet manager. If ```load_manager``` is
  ```true```, a nodelet manager is created with this name. If ```load_manager```
  is ```false```, the nodelet manager with this name is used. Default:
  ```camera_manager```.
- ```manager_threads``` (integer): The number of threads used by the nodelet manager, if
  created. Default: ```4```.
- ```launch_dynamic_reconfigure``` (boolean): Whether or not ```rqt_reconfigure```
  should be launched (One instance of ```rqt_reconfigure``` is enough to
  control all nodes that use it, including all cameras). Default: value of
  ```load_manager```.

## Configuration files
Configuration files are located in the ```config``` directory. They contain configuration information for different cameras, inculding sensor information used by the GenICam standard and distortion/rectification/rectifcation matrices.

Input/Output parameters:
- ```guid```: The name of the camera, as seen by aravis/GenICam. The output of
  running ```arv-tool-0.6``` will suffice.
- ```publish_tf```: Whether or not transformation data should be published.
- ```tf_publish_rate```: The rate at which transformation data should be
  published.

GenICam settings (if ommited, values will be detected automatically). See the
[GenICam SNFC](https://www.emva.org/wp-content/uploads/GenICam_SFNC_2_0_0.pdf)
for more infomtaion.

Camera Calibration Info. Gotten from running
```rosrun camera_calibration cameracalibrator.py``` and copy/pasting the
contents of the resulting ```.yml``` file into this config file. See the
[```camera_calibration```](https://wiki.ros.org/camera_calibration) package for
more info.