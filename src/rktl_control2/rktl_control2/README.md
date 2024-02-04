# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun rktl_control <Node name>
```

```{contents} ROS Nodes in the package
:depth: 2
:backlinks: top
:local: true
```

---

## controller

Customizable controller for the car. It implements either a PID controller or a
lead-lag controller.

The controller is responsible for making the car do what you tell it to do.
Simply, that means you give it a
[`ControlCommand.msg`](/rktl_msgs/html/msg/ControlCommand.html#http://) and it
produces a [`ControlEffort.msg`](/rktl_msgs/html/msg/ControlEffort.html#http://)
that makes the car follow it. At a basic level, it uses the odometry produced by
the filter and tries to minimize the error between what you want the car to do
and what it is actually doing.

The incoming message is different from the outgoing message because it is taking
a desired motion that could be for any mechanism, and it is converting it to
very specific hardware commands for our specific cars. This allows the software
generating the commands to ignore the specifics of the cars, and the software /
hardware getting the efforts to the actuators to also ignore the specifics of
the car as a whole.

The two types of controllers supported are PID and lead-lag. If you've never
heard of these before, PID is going to be something you can intuitively
understand with some work, and lead-lag is probably something you're going to
need to take some classes or read some textbooks to understand (ex: Purdue ME
365, 375, 475).

PID control is very well explained in many places on the internet, so I won't
discuss the basics here. The reason it is an option is because it is a commonly
used controller that can be tuned intuitively by watching how the car is
behaving. Lead-Lag is a very similar algorithm that can result in better
performance (especially for discrete systems like this), but it is harder to
tune. Intuitively tuning likely isn't an option, so instead MATLAB scripts can
be really helpful to determine the specific gains to use. These are included in
the `scripts` directory of this package.

Some possibly useful references:
- <https://www.youtube.com/watch?v=UR0hOmjaHp0>
- <https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y>
- <https://www.youtube.com/watch?v=xLhvil5sDcU>

These are all by Brian Douglas, who has many useful videos on control concepts.

### Subscribed Topics

- `command` ([rktl_msgs/ControlCommand](/rktl_msgs/html/msg/ControlCommand.html#http://)):
    The car's desired movement, in terms of velocity and steering angle.
- `odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry data of the car's position and velocity.

### Published Topics

- `effort` ([rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html#http://)):
    The car's desired movement, in terms of throttle and steering amount.

### Parameters

- `/cars/throttle/max_speed` (float): Maximum velocity for the car, in meters
    per second.
- `/cars/steering/max_throw` (float): Maximum steering angle for the car, in
    meters per second.
- `/cars/length` (float): Length of the car, in meters.
- `~limits/throttle/min` (float, default: `-1.0`): Min value for the car throttle.
- `~limits/throttle/max` (float, default: `1.0`): Max value for the car throttle.
- `~limits/steering/min` (float, default: `-1.0`): Min value for the car steering.
- `~limits/steering/max` (float, default: `1.0`): Max value for the car steering.
- `~open_loop/publish_early` (boolean, default: true): If true, data on the 
    `effort` topic will be published before the controller updates its internal
    state. This means that the control effort message will be based on the
    previous error and controller output values, rather than current error and
    controller output values.
- `~controller_type` (string): One of `'lead_lag'`, `'pid'`, or `'none'`. Sets
    the controller type to use a lead-lag controller, a PID controller, or no
    controller, respectively.
- `~lead/gain` (float): Parameter for the lead-lag controller.
- `~lead/alpha` (float): Parameter for the lead-lag controller.
- `~lead/beta` (float): Parameter for the lead-lag controller.
- `~lag/alpha` (float): Parameter for the lead-lag controller.
- `~lag/beta` (float): Parameter for the lead-lag controller.
- `~pid/kp` (float): Parameter for the PID controller.
- `~pid/ki` (float): Parameter for the PID controller.
- `~pid/kd` (float): Parameter for the PID controller.
- `~pid/anti_windup` (float): Parameter for the PID controller.
- `~pid/deadband` (float): Parameter for the PID controller.
- `~rate` (float, default: `10.0`): Rate at which the `effort` topic is
    published, in messages per second.

---

## keyboard_interface

Allows for controlling the car using a keyboard interface. Uses the terminal
(with ncurses) as an interface.

### Published Topics

- `effort` ([rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html#http://)):
    The car's desired movement, in terms of throttle and steering amount.

---

## mean_odom_filter

This is a very simple filter. If you have a stream of noisy measurements,
averaging the most recent $N$ measurements will give you a pretty good estimate.
This gets slightly complicated when handling angles, since they wrap around at
$\pm\pi$, but it still works.

The downside is that if you average 10 measurements, your estimate is
time-delayed by 5 sampling periods. Therefore, a balance needs to be struck
between the desired noise reduction and the acceptable delay.

This filter also runs into problems when taking derivatives of the input. Simply
taking the derivative of the two most recent measurements, filtered or
unfiltered, will result in significant noise. To combat this, the filter computes
derivatives for each pair of points (ex: $t-1$ and $t-2$, $t-2$ and $t-3$, $t-3$
and $t-4$), then averages those velocity predictions using the same averaging
buffer algorithm.

Overall, this filter is simple and works well enough for position as long as slight delay is acceptable. For velocity, it runs into problems since the noise is greater and the sampling frequency isn’t high enough to get a low delay signal.

### Subscribed Topics

- `pose_sync` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    The position, rotation, and timestamp of a given element on the field,
    synchronized across all cameras.

### Published Topics

- `odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)):
    Odometry data of a field element's position and velocity.

### Parameters

- `~frame_ids/map` (string, default: `'map'`): Frame ID of the common reference
    frame for all objects. In the context of the project, this is the frame
    centered on the middle of the field, with the $x$-axis pointed towards a goal,
    the $y$-axis pointing towards a sideline, and the $z$-axis pointed up. By
    convention, this should be called `'map'`.
- `~frame_ids/body` (string, default: `'base_link'`): Frame ID of the object
    being tracked.
- `~buffer_size/position` (int, default: `3`): Buffer size for the rolling
    average for position.
- `~buffer_size/velocity` (int, default: `3`): Buffer size for the rolling
    average for velocity.
- `~delay/compensate` (boolean, default: false): If true, the published odom is
    extrapolated to a future time defined by `~delay/duration`.
- `~delay/duration` (float, default: `0.0`): The time, in seconds, in the future
    that should be predicted. Requires `~delay/compensate` to be true to have
    any effect.

---

## particle_odom_filter

This is a much more complex algorithm. The basic premise of why the algorithm is
useful is that by considering the noise in each individual measurement, not just
the mean value, there is more detail that can be used to improve the accuracy vs
delay trade off. Several similar algorithms were considered and are briefly
discussed below.

The basic premise is that a particle filter has many different estimates of what
the car is doing and where it is (called a particle; perhaps 1,000 in this
application). For each new measurement that comes in:

- Each and every one of these particles are simulated one time step (using a
    bicycle model) to get a prediction of the current state of the particle
- For each particle, the question is asked, "Given this measurement and what I
    know about it's standard deviation, what is the probability that this
    particle is correct?"
- That result is used to compute a weighed average of all the particles.
- This average is then post processed a little bit to produce an odometry
    estimate since the internal states don't exactly match

This then repeats until the end of time.

Initially, the particles are randomly distributed all over the field, but they
eventually converge (pretty quickly) to the real state of the car. To do this,
the filter will periodically delete bad particles, and replace them with new
random ones. These random ones are uniformly distributed if there isn't a good
guess of where the car is (like when the filter first starts), or distributed
normally about the most recent guess

The specific state of the car is tracked by five-ish variables:

- $x$, the $x$ location of the car's center of mass in the field's reference frame
- $y$, the $y$ location of the car's center of mass in the field's reference frame
- $\sin(\theta)$, the $y$ component of the car's heading angle in the field's
    reference frame
- $\cos(\theta)$, the $x$ component of the car's heading angle in the field's
    reference frame
- $v_{rear}$, the linear velocity of the rear tires of the car
- $\psi$, the steering angle (relative to the car's center)

$\theta$ is split into two state variables to prevent issues with wrap around
and summation.

From these, a kinematic bicycle model is used to calculate out everything needed
for an [odometry message](/nav_msgs/html/msg/Odometry.html#http://). Basically, this
just assumes there is no side-slip by the tires and it uses a lot of
trigonometry. Details are in the MATLAB script [`scripts/bicycle_model.m`].

When advancing each particle one time-step, either the known control vector
(from listening to controller output) or a random control vector is used. If the
known vector is used, a small amount of random noise is added. If a completely
random one is used, they are uniformly distributed between actuator saturation
limits. From these, a new wheel velocity and steering angle are predicted
(using a 1st order and 0 order velocity model) and fed into the bicycle model.

All parameters to the filter are defined in `config/particle_odom_filter.yaml`.
The relevant ones to tweak are:

[Here](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb)
is a good resource on particle filters for more information.

### Subscribed Topics

- `pose_sync` ([geometry_msgs/PoseWithCovarianceStamped](/geometry_msgs/html/msg/PoseWithCovarianceStamped.html#http://)):
    The position, rotation, and timestamp of the car, synchronized across all
    cameras.
- `effort` ([rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html#http://)):
    The car's desired movement, in terms of throttle and steering amount.

### Published Topics

- `odom` ([nav_msgs/Odometry](/nav_msgs/html/msg/Odometry.html#http://)): Odometry data
    of a field element's position and velocity.
- `odom_particles` ([geometry_msgs](/geometry_msgs/html/msg/PoseArray.html#http://)):
    Poses of all particles used in the filter. Useful for debugging. Only
    published if `~publish_particles` is true.

### Parameters

- `/field/width` (float): Width of the playing field, in meters.
- `/field/length` (float): Length of the playing field, in meters.
- `/cars/length` (float): Length of the car, in meters.
- `/cars/throttle/max_speed` (float): Maximum speed of the car, in meters per
    second.
- `/cars/throttle/tau` (float): The throttle time constant, in seconds.
- `/cars/steering/max_throw` (float): The maximum steering throw, in radians.
- `/cars/steering/rate` (float): The maximum steering rate, in radians per
    second.
- `~frame_ids/map` (string, default: `'map'`): Frame ID of the common reference
    frame for all objects. In the context of the project, this is the frame
    centered on the middle of the field, with the $x$-axis pointed towards a goal,
    the $y$-axis pointing towards a sideline, and the $z$-axis pointed up. By
    convention, this should be called `'map'`.
- `~frame_ids/body` (string, default: `'base_link'`): Frame ID of the object
    being tracked.
- `~rate` (float, default: `10.0`): Rate at which the `effort` topic is
    published, in messages per second.
- `~supersampling` (int, default: `1`): Supersampling steps in the particle
    dynamics calculation.
- `~publish_particles` (boolean, default: false): If true, poses of all
    particles used in the filter are published on the `odom_particles` topic.
- `~allowable_latency` (float, default: `1.2`): Allowable latency used for the
    watchdog timer for dropped pose messages.
- `~open_loop_limit` (int, default: `10`): Used in the watchdog timer for
    dropped pose messages
- `~delay/compensate` (boolean, default: false): If true, the published odom is
    extrapolated to a future time defined by `~delay/duration`.
- `~delay/duration` (float, default: `0.0`): The time, in seconds, in the future
    that should be predicted. Requires `~delay/compensate` to be true to have
    any effect.
- `~boundary_check` (boolean, default: false): If true, the filter artificially
    punishes particles outside of the assumed field boundaries.
- `~num_particles` (int, default: `1000`): Number of particles used in the
    filter. A higher number of particles should increase the accuracy of
    the filter, but will consume higher computational resources.
- `~resample_proportion` (float, default: `0.1`): The percentage of particles
    replaced with random guesses at every timestep.
- `~measurement_error/location` (float, default: `0.05`): Assumed standard
    deviation of the perception data coming in.
- `~measurement_error/orientation` (float, default: $5^\circ$): Assumed standard
    deviation of the perception data coming in.
- `~generator_noise/location` (float, default: `0.05`): Standard deviations of
    the gaussian noise added.
- `~generator_noise/orientation` (float, default: `0.05`): Standard deviations
    of the gaussian noise added.
- `~generator_noise/velocity` (float, default: `0.05`): Standard deviations of
    the gaussian noise added.
- `~generator_noise/steering_angle` (float, default: $1^\circ$): Standard
    deviations of the gaussian noise added.
- `~efforts/enable` (boolean, default: false): If true, the filter uses historic
    effort data to get a more accurate idea of where the car is.
- `~efforts/buffer_size` (int, default: `0`): Buffer size of the buffer for
    effort messages.
- `efforts/throttle/noise` (float, default: `0.05`): Standard deviation of
    gaussian noise added.
- `efforts/steering/noise` (float, default: `0.05`): Standard deviation of
    gaussian noise added.
- `~efforts/throttle/max` (float, default: `1.0`): Max value for the car throttle.
- `~efforts/throttle/min` (float, default: `-1.0`): Min value for the car throttle.
- `~efforts/steering/max` (float, default: `1.0`): Max value for the car steering.
- `~efforts/steering/min` (float, default: `-1.0`): Min value for the car steering.

---

## pose_synchronizer

Downsample the raw perception data to produce synchronized poses.

For every topic defined by `~topics`, that topic is subscribed to and
republished at a rate defined by `~rate`. The name of the topic published
is the original name of the topic with `_sync` appended to the end.

## Parameters

- `~map_frame` (string, default: `'map'`): Frame ID of the common reference
    frame for all objects. In the context of the project, this is the frame
    centered on the middle of the field, with the $x$-axis pointed towards a goal,
    the $y$-axis pointing towards a sideline, and the $z$-axis pointed up. By
    convention, this should be called `'map'`.
- `~topics` (array): Array of topic names to synchronize.
- `~rate` (float, default: `10.0`): Rate at which the synchronized topics are
    published, in messages per second.
- `~delay` (float, default: `0.15`): Rate at which the synchronized topics are
    published, in messages per second.
- `~publish_latency` (boolean, default: false): If true, the latency from
    synchronizing each topic is also published
- `~use_weights` (array) :Unused

---

## topic_delay

Delay an arbitrary ROS topic without modifying message

---

## xbox_interface

Allows for controlling the car using a joystick input, e.g. an Xbox controller.

### Subscribed Topics

- `joy` ([sensor_msgs/Joy](/sensor_msgs/html/msg/Joy.html#http://)): State of
    all buttons and axes on the Xbox controller.


### Published Topics

- `effort` ([rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html#http://)):
    The car's desired movement, in terms of throttle and steering amount.

### Parameters

- `~base_throttle` (float): Throttle when not boosting.
- `~boost_throttle` (float): Throttle when boosting.
- `~cooldown_ratio` (float): How long should one second of boost take to recharge?
- `~max_boost` (float): Max time boost can be active.

---
