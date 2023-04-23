# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun rktl_sim <Node name>
```

:::{contents} ROS Nodes in the package
:depth: 2
:backlinks: top
:local: true
:::

---

## simulator

This node wraps around a physics simulator and allows controlling a car
in a simulated environment. The ROS parameter server is used to configure the
simulator, and publishes sensor data and control commands as ROS messages.
The simulator is implemented in `simulator.py` in the `src` directory. It can be
run in `'realistic'` or `'ideal`' mode by changing the `~mode` parameter. If the
mode is `'ideal'`, the simulator will use perfect sensor readings and ignore
any control effort messages that it receives. If the mode is `'realistic'`,
the simulator will add noise to sensor readings and simulate the effects of
control effort on the car's motion. Many parameters are set in
`rktl_launch/config.global_params.yaml` or `rktl_sim/config/simulator.yaml`.

### Subscribed Topics

- `/cars/car0/effort` ([rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html#http://)):
    The car's desired movement, in terms of throttle and steering amount. Only
    subscribed to if the `~mode` parameter is set to `'realistic'`.
- `/cars/car0/command` ([rktl_msgs/ControlCommand](/rktl_msgs/html/msg/ControlCommand.html#http://)):
    The car's desired movement, in terms of velocity and steering angle. Only
    subscribed to if the `~mode` parameter is set to `'ideal'`.

### Published Topics

- `/ball/pose_sync_early` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    Pose of the ball with a timestamp and added noise. Only published if the
    `~mode` parameter is set to `'realistic'`.
- `/ball/odom_truth` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry of the ball with no added noise. Only published if the `~mode`
    parameter is set to `'realistic'`.
- `/ball/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry of the ball with no added noise. Only published if the `~mode`
    parameter is set to `'ideal'`.
- `/cars/car0/pose_sync_early` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    Pose of the car with a timestamp and added noise. Only published if the
    `~mode` parameter is set to `'realistic'`.
- `/cars/car0/odom_truth` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry of the car with no added noise. Only published if the `~mode`
    parameter is set to `'realistic'`.
- `/cars/car0/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry of the car with no added noise. Only published if the `~mode`
    parameter is set to `'ideal'`.
- `match_status` ([rktl_msgs/MatchStatus](/rktl_msgs/html/msg/MatchStatus.html#http://)):
    Status of the match, whether the match is ongoing or if a goal has been
    scored, ending the match (and which goal it was scored on).

### Parameters

- `~mode` (string): Mode of the simulator (either '`ideal'` or `'realistic'`).
- `~render` (bool, default: False): Whether or not to enable rendering of the
    simulation.
- `~rate` (int, default: 30): Rate at which to run the simulation.
- `~frame_id` (string, default: 'map'): Frame ID to use for published ROS
    messages.
- `~timeout` (int, default: 10): Timeout in seconds for certain simulator
    functions.
- `/field/width` (float): Width of the playing field, in meters.
- `/field/length` (float): Length of the playing field, in meters.
- `/field/wall_thickness` (float): Thickness of the walls around the playing
    field, in meters.
- `/field/goal/width` (float): Width of the goals, in meters.
- `~spawn_height` (float, default: 0.06): Height at which to spawn the ball and
    car.
- `~urdf` (dict): Dictionary of URDF file paths for each robot. Can be set by
    setting `urdf/*` to whatever is needed.
- `~ball/init_pose` (list): List of 3 floats representing the initial position
    of the ball.
- `~ball/init_speed` (list): List of 3 floats representing the initial speed of
    the ball.
- `~ball/sensor_noise` (dict): Dictionary of noise parameters for the ball's
    sensors.
- `/cars/length` (float): Length of the car, front to rear wheel center to
    center, in meters.
- `/cars/throttle/max_speed` (float): Maximum speed of the car, in meters per
    second.
- `/cars/throttle/tau` (float): The throttle time constant, in seconds.
- `/cars/steering/max_throw` (float): The maximum steering throw, in radians.
- `/cars/steering/rate` (float): The maxmimum steering rate, in radians per
    second.
- `~car/init_pose` (list): List of 3 floats representing the initial position
    of the car.
- `~car/sensor_noise` (dict): Dictionary of noise parameters for the car's
    sensors.

---

## visualizer

This node provides a visualizer that uses Pygame to show the position of a car
and a ball on a playing field. The script subscribes to topics that provide
information about the position of the car and the ball, and updates their
position in the visualizer accordingly. Additionally, the script can display a
planned path for the car using a linear path or a bezier curve. Many parameters
are set in `rktl_launch/config.global_params.yaml` or
`rktl_sim/config/visualizer.yaml`.

### Subscribed Topics

- `/ball/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    The ball's odometry information containing its position, orientation, and velocities.
- `/cars/car0/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    The car's odometry information containing its position, orientation, and velocities.
- `/cars/car0/path` ([rktl_msgs/Path](/rktl_msgs/html/msg/Path.html#http://)):
    Used for visualizing the path the car should follow as a series of waypoints.
- `/cars/car0/lookahead_pnt` ([std_msgs/Float32](/std_msgs/html/msg/Float32.html#http://)):
    Used for visualizing the path the lookahead point for a pure pursuit algorithm.
- `/agents/agent0/bezier_path` ([rktl_msgs/BezierPathList](/rktl_msgs/html/msg/BezierPathList.html#http://)):
    Used for visualizing the path the car should follow as a bezier curve.

### Parameters

- `/field/width` (float): Width of the playing field, in meters.
- `/field/length` (float): Length of the playing field, in meters.
- `/field/wall_thickness` (float): Thickness of the walls around the playing
    field, in meters.
- `/field/goal/width` (float): Width of the goals, in meters.
- `/ball/radius` (float): Radius of the ball, in meters.
- `window_name` (string, default: 'Rocket League Visualizer'): Name of the
    window
- `~frame_id` (string, default: 'map'): Frame ID to use for published ROS
    messages.
- `~timeout` (int, default: 10): Timeout in seconds for certain simulator
    functions.
- `~rate` (int, default: 30): Rate at which to run the simulation.
- `~cars/body_width` (float): Rendered width of the car in the visualizer, in
    meters.
- `~cars/body_length` (float): Rendered length of the car in the visualizer,
    in meters.
- `~media/field` (string): Path to the image representing the field.
- `~media/car` (string): Path to the image representing the car.
- `~media/ball` (string): Path to the image representing the ball.

---
