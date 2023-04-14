# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun rktl_planner <Node name>
```

**Nodes:**
- [`bezier_path_server`](#bezier-path-server)
- [`path_follower`](#path-follower)
- [`path_planner`](#path-planner)
- [`patrol_planner`](#patrol-planner)

---

## bezier_path_server

This node is a ROS service that creates a Bezier path using a series of input
poses, target durations, and segment durations. The output includes the Bezier
path (which is a series of connected Bezier curves) and a linear path (which is
a series of linear segments that approximate the Bezier path). The Bezier path
is designed to allow for smooth turns and gradual acceleration and deceleration,
while the linear path is used for easier computation and execution.

### Subscribed Topics:
This node does not subscribe to any topics.

### Published Topics:
This node does not publish any topics.

### Services:
- `create_bezier_path` (rktl_planner/CreateBezierPath): a service that
    creates a Bezier path using the input poses and durations.

### Parameters:
This node does not have any parameters.

---

## path_follower

This node ensures the car follows a given trajectory using the pure pursuit
algorithm. It subscribes to the `/cars/{car_name}/odom` topic to obtain the
car's odometry information and the `/linear_path` topic to receive the desired
path. It then publishes the control commands to the `/cars/{car_name}/command`
topic.

### Subscribed Topics:
- `/cars/{car_name}/odom` (nav_msgs/Odometry): The car's odometry information
    containing its position, orientation, and velocities.
- `linear_path` (rktl_msgs/Path): The desired path that the car should follow.

### Published Topics:
- `/cars/{car_name}/command` (rktl_msgs/ControlCommand): The control commands to
    be sent to the car to follow the desired path

### Parameters:
- `~frame_id` (str, default: 'map'): The frame ID for the path and pose messages.
- `~max_speed` (float, default: 0.1): The maximum speed at which the car should 
    travel along the path.
- `~lookahead_dist` (float, default: 0.15): The distance ahead of the car to
    look for the next point on the path.
- `~lookahead_gain` (float, default: 0.035): The coefficient used to adjust the
    lookahead distance based on the car's current speed.
- `~lookahead_pnts` (int, default: -1): The number of points to search along the
    path for the next valid intersection. If set to -1, the entire path is searched.
- `~car_name` (str): The name of the car to which this node belongs.

---

## path_planner

This node uses `bezier_path_server` to generate bezier paths between a car and
the ball in order to hit the ball into a goal. Options exist for generating a
'simple' or 'complex' path to the ball. A 'simple' path is a direct path between
the current position of the car and the ball's position, while a 'complex' path
takes into account the orientation of the car and decides whether to take a
direct path or to generate a path in reverse.

### Subscribed Topics:
- `/cars/{car_name}/odom` (nav_msgs/Odometry): The car's odometry information
    containing its position, orientation, and velocities.

### Published Topics:
- `linear_path` (rktl_msgs/Path): The desired path that the car should follow,
    split into linear segments
- `bezier_path` (rktl_msgs/BezierPath): The desired path that the car should
    follow, split into bezier segments

### Services:
- `reset_planner` (std_srvs/Empty): An empty service that is used to trigger
    the generation/regeneration of the path based on the odemetry of the car
    and ball. The new path is published on the topics above.

### Parameters:
- `~frame_id` (str, default: 'map'): The frame ID for the path and pose messages.
- `~max_speed` (float, default: 0.1): The maximum speed at which the car should 
    travel along the path.
- `~lookahead_dist` (float, default: 0.15): The distance ahead of the car to
    look for the next point on the path.
- `~lookahead_gain` (float, default: 0.035): The coefficient used to adjust the
    lookahead distance based on the car's current speed.
- `~lookahead_pnts` (int, default: -1): The number of points to search along the
    path for the next valid intersection. If set to -1, the entire path is searched.
- `~car_name` (str): The name of the car to which this node belongs.

---

## patrol_planner

This node implements a "patrol"-based algorithm to control the car. The car
"patrols" around the field in a circle and makes a beeline towards the ball if
it seems like it would result in a goal. Logic is in place for both offensive
and defensive modes. A Proportional Derivative (PD) controller is used to
determine how much to turn.

### Subscribed Topics:
- `odom` (nav_msgs/Odometry): The car's odometry information, containing its
    position, orientation, and velocities.
- `/ball/odom` (nav_msgs/Odometry): The ball's odometry information, containing
    its position, orientation, and velocities.

### Published Topics:
- `command` (rktl_msgs/ControlCommand): The control commands to be sent to the
    car to perform the patrol algorithm

### Parameters:
- `/field/width` (float): Physical constant, width of the field
- `/field/length` (float): Physical constant, length of the field
- `/cars/steering/max_throw` (float): Physical constant, maximum steering throw
- `/cars/length` (float): Physical constant, car length
- `~speed` (float, default: 1.0): Maximum speed of the car
- `~curvature_gain/kp` (float, default: 1.0): Parameter for PD Controller
- `~curvature_gain/kd` (float, default: 0.0): Parameter for PD Controller
- `~curvature_gain/falloff` (float, default: 1e-9): Parameter for PD Controller
- `~patrol_wall_dist` (float, default: 0.5)
- `~wall_avoidance/reverse_time` (float, default: 0.25)
- `~wall_avoidance/distance_margin` (float, default: 0.5)
- `~wall_avoidance/heading_margin` (float, default: π/4)
- `~attempt_timeout` (float, default: 5.0)
- `~defensive_line` (float: 0.0)
- `~reverse_line` (float: 0.0)
- `~scoring/heading_margin` (float, default: π/8)
- `~scoring/car_lookahead_dist` (float, default: 1.0)
- `~scoring/ball_lookahead_time` (float, default: 0.0)
- `~scoring/goal_depth_target` (float, default: 0.0)
- `~defense/reverse_time_gain` (float, default: 0.5)

---
