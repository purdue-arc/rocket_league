# Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```shell
rosrun rktl_autonomy <Node name>
```

```{contents} ROS Nodes in the package
:depth: 2
:backlinks: top
:local: true
```

---

## cartpole_env

This node creates a connection between the CartPole-v0 environment from OpenAI's
Gym library and ROS for testing and simulation purposes. It subscribes to the
`cartpole/action` topic, which receives a message containing an action (an
integer) that is taken by the agent. It also publishes messages to the
`cartpole/observation`, `cartpole/reward`, and `cartpole/done` topics, which
contain the current observation (a Float32MultiArray), the current reward (a
Float32), and a Boolean indicating whether the episode is done or not,
respectively.

### Subscribed Topics

- `cartpole/action` ([std_msgs/Int32](/std_msgs/html/msg/Int32.html#http://)):
    The action that should be performed.

### Published Topics

- `cartpole/observation` ([std_msgs/Float32MultiArray](/std_msgs/html/msg/Float32MultiArray.html#http://)):
    Observation of the system at any given point.
- `cartpole/reward` ([std_msgs/Float32](/std_msgs/html/msg/Float32.html#http://)):
    Reward for training.
- `cartpole/done` ([std_msgs/Bool](/std_msgs/html/msg/Bool.html#http://)):
    Boolean representing the whether or not the simulation is complete.

---

## plotter

This node plots the progress of a machine learning algorithm. The progress is
subscribed to via the `diagnostic_msgs/DiagnosticStatus` message. The node uses
the `mpl_toolkits` library to plot the progress of the training, which is then
saved as a PNG image file.

### Subscribed Topics

- `~log` ([diagnostic_msgs/DiagnosticStatus](/diagnostic_msgs/html/msg/DiagnosticStatus.html#http://)):
    Log for the training algorithm.

### Parameters

- `~log/base_dir` (string): The base directory where the training log files are
    stored.
- `~log/plot_freq` (int, default: `25`): The frequency of plotting, i.e., how
    often the node updates the plot with new data.
- `~log/basic` (list, default: `['duration']`): A list of basic variables to
    plot. These variables have only one line on the plot.
- `~log/advanced` (list, default: `['net_reward']`): A list of advanced
    variables to plot. These variables have three lines on the plot: the
    average, the minimum, and the maximum.
- `~frequency` (float, default: `1/30`): Frequency of the main simulation loop.

---

## rocket_league_agent

This node runs the agent trained for the Rocket League project. It uses the
`RocketLeagueInterface` class in `rocket_league_interface.py`.

### Subscribed Topics

- `cars/car0/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry data of the car's position and velocity.
- `ball/odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry data of the car's position and velocity.
- `match_status` ([rktl_msgs/MatchStatus](/rktl_msgs/html/msg/MatchStatus.html#http://)):
    Match status data containing the current score of the game.

### Published Topics

- `/cars/car0/command` ([rktl_msgs/ControlCommand](/rktl_msgs/html/msg/ControlCommand.html#http://)):
    ontrol commands for the car.

### Parameters

- `~weights` (string): The location of weights for where the model was saved.
- `/cars/throttle/max_speed` (float): Maximum velocity for the car, in meters
    per second.
- `/cars/steering/max_throw` (float): Maximum steering angle for the car, in
    meters per second.
- `~action_space/velocity/min` (float): Optional. Action space override for
    velocity for the car, in meters per second.
- `~action_space/velocity/max` (float): Optional. Action space override for
    velocity for the car, in meters per second.
- `~action_space/curvature/min` (float): Optional. Action space override for
    steering angle for the car, in meters per second.
- `~action_space/curvature/max` (float): Optional. Action space override for
    steering angle for the car, in meters per second.
- `/cars/length` (float): Length of the car, in meters.
- `/field/width` (float): Width of the field, in meters.
- `/field/length` (float): Length of the field, in meters.
- `~observation/goal_depth` (float, default, `0.075`): Depth of the goal, in
    meters.
- `~observation/velocity/max_abs` (float, default, `3.0`): Max absolute velocity
    for observation. (default is 3.0)
- `~observation/angular_velocity/max_abs` (float, default: `2π`): Max absolute
    angular velocity for observation. (default is 2*pi)
- `~max_episode_time` (float, default: `30.0`): Maximum time in seconds for an
    episode.
- `~reward/constant` (float, default: `0.0`): Constant reward for each step.
- `~reward/ball_dist_sq` (float, default: `0.0`): Reward for getting closer to
    the ball.
- `~reward/goal_dist_sq` (float, default: `0.0`): Reward for getting closer to
    the goal.
- `~reward/direction_change` (float, default: `0.0`): Reward for changing the
    direction.
- `~reward/win` (float, default: `100.0`): Reward for winning.
- `~reward/loss` (float, default: `0.0`): Reward for losing.
- `~reward/reverse`:  (float, default: `0.0`)Reward for reversing.
- `~reward/walls/value` (float, default: `0.0`): Reward for hitting walls.
- `~reward/walls/threshold` (float, default: `0.0`): Distance from the wall at
    which reward is given.

---

## snake_agent

This node runs the agent trained for the snake environment. It uses the
`SnakeInterface` class in `snake_interface.py`.

### Subscribed Topics

- `snake/pose` ([geometry_msgs/PoseArray](/geometry_msgs/html/msg/PoseArray.html#http://)):
    Pose of the snake parts.
- `snake/goal` ([geometry_msgs/PointStamped](/geometry_msgs/html/msg/PointStamped.html#http://)):
    Location of the apple.
- `snake/score` ([std_msgs/Int32](/std_msgs/html/msg/Int32.html#http://)):
    Current Score.
- `snake/active` ([std_msgs/Bool](/std_msgs/html/msg/Bool.html#http://)):
    Whether or not the snake is active.

### Published Topics

- `snake/cmd_vel` ([geometry_msgs/Twist](/geometry_msgs/html/msg/Twist.html#http://)):
    Command for the snake to follow.

### Parameters

- `~num_segments` (int, default: 7): Initial number of segments of the snake.
- `~field_size` (float, default: 10): Size of the field.
- `~control/max_angular_velocity` (float, default: 3.0): Max angular velocity.
- `~control/max_linear_velocity` (float,  default: 3.0): Max velocity.
- `~reward/death` (float, default: 0.0): Penalty for death.
- `~reward/goal` (float, default: 50.0): Reward for eating an apple.
- `~reward/distance` (float, default: 0.0): Reward for distance traveled.
- `~reward/constant` (float, default: 0.0): Constant reward.
- `~max_episode_time` (float, default: 30.0): Maximum time in seconds for an
    episode.

---
