# Rocket League Control
This package performs several jobs that allow it to move physical cars according
to commands from either the planning or autonomy code.

Specifically it has three main components:
- It filters the raw perception data using either a moving average filter or a
particle filter, while also producing estimated odometry (which includes velocity
in addition to the position given by the raw perception data)
- It has a closed loop controller which reduces the error between the desired and
estimated motion of the car by producing control signals.
- It has a hardware interface, which sends those control signals to the physical cars.

It also contains several MATLAB scripts that are useful in tuning and validating
the Python code used for this project.

## Filters
### Moving Average
This is a very simple filter. If you have a stream of noisy measurements,
averaging the most recent N measurements will give you a pretty good estimate.
This gets slightly complicated when handling angles, since they wrap around at
+/- pi, but it still works.

The downside is that if you average 10 measurements, your estimate is time-delayed
by 5 sampling periods. Therefore, a balance needs to be struck between the desired
noise reduction and the acceptable delay.

This filter also runs into problems when taking derivatives of the input.
Simply taking the derivative of the two most recent measurements, filtered or
unfiltered, will result in significant noise. To combat this, the filter computes
derivatives for each pair of points (ex: `t-1` and `t-2`, `t-2` and `t-3`, `t-3` and `t-4`),
then averages those velocity predictions using the same averaging buffer algorithm.

The relevant parameters to the filter (defined in [`mean_odom_filter.yaml`](config/mean_odom_filter.yaml))
are `buffer_size/position` and `buffer_size/velocity`. Note that the velocity
buffer size is the number of velocity measurements, not position measurements.
Therefore, the number of position measurements in the buffer is one greater.

It also has predictive abilities, which extrapolate the current position and
orientation to the future. If used, this should be configured so that it outputs
an estimate of what the world will be like when the currently being calculated
controls arrive. This is controlled through the `delay` parameters.

Overall, this filter is simple and works well enough for position as long as
slight delay is acceptable. For velocity, it runs into problems since the noise
is greater and the sampling frequency isn't high enough to get a low delay signal.

### Particle Filter
This is a much more complex algorithm. The basic premise of why the algorithm is
useful is that by considering the noise in each individual measurement, not just
the mean value, there is more detail that can be used to improve the accuracy vs
delay trade off. Several similar algorithms were considered and are briefly
discussed below.

The basic premise is that a particle filter has many different estimates of what
the car is doing and where it is (called a particle; perhaps 1,000 in this
application). For each new measurement that comes in:
- Each and every one of these particles are simulated one time step (using a bicycle
model) to get a prediction of the current state of the particle
- For each particle, the question is asked, "Given this measurement and what I
know about it's standard deviation, what is the probability that this particle
is correct?"
- That result is used to compute a weighed average of all the particles.
- This average is then post processed a little bit to produce an odometry estimate
since the internal states don't exactly match

This then repeats until the end of time.

Initially, the particles are randomly distributed all over the field, but they
eventually converge (pretty quickly) to the real state of the car. To do this,
the filter will periodically delete bad particles, and replace them with new
random ones. These random ones are uniformly distributed if there isn't a good
guess of where the car is (like when the filter first starts), or distributed
normally about the most recent guess

The specific state of the car is tracked by five-ish variables:
- x, the x location of the car's center of mass in the field's reference frame
- y, the y location of the car's center of mass in the field's reference frame
- sin(theta), the Y component of the car's heading angle in the field's reference frame
- cos(theta), the X component of the car's heading angle in the field's reference frame
- v_rear, the linear velocity of the rear tires of the car
- psi, the steering angle (relative to the car's center)

Theta is split into two state variables to prevent issues with wrap around and
summation.

From these, a kinematic bicycle model is used to calculate out everything needed
for an odometry message. Basically, this just assumes there is no side-slip by
the tires and it uses a lot of trigonometry. Details are in the MATLAB script
[`bicycle_model.m`](scripts/bicycle_model.m).

When advancing each particle one time-step, either the known control vector (
from listening to controller output) or a random control vector is used. If the
known vector is used, a small amount of random noise is added. If a completely
random one is used, they are uniformly distributed between actuator saturation
limits. From these, a new wheel velocity and steering angle are predicted
(using a 1st order and 0 order velocity model) and fed into the bicycle model.

All parameters to the filter are defined in [`particle_odom_filter.yaml`](config/particle_odom_filter.yaml).
The relevant ones to tweak are:
- `boundary_check`: Should the filter artificially punish particles outside of
the assumed field boundaries?
- `delay`: The `total_cycles` parameter should be the number of delta_t sized
delays between an actuation being sent to the vehicle and perception seeing it
move. The `compensate` parameter means if the filter should try to predict the
future state of the field, corresponding to when the currently being calculated
controls would arrive.
- `num_particles`: A higher number of particles should increase the accuracy of
the filter, but will consume higher computational resources.
- `resample_proportion`: The percentage of particles replaced with random guesses
at every timestep.
- `measurement_error`: These are the assumed standard deviations of the
perception data coming in.
- `generator_noise`: These are the standard deviations of the gaussian noise
added to the most recent state estimate when producing random new particles.
- `efforts`: The `max` and `min` values are actuator saturation limits. This
should match what the controller is allowed to produce. The `noise` value is the
standard deviation of gaussian noise added if the controls are known.

A good resource I found on particle filters is here: <https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb>

#### Alternative algorithms
[Kalman filters](https://www.kalmanfilter.net/default.aspx) were also considered.
This is similar to a particle filter in that it is cognizant of measurement noise
to improve its prediction, but it estimates the state using a series of means
and covariances. This restricts it to a normal distribution, whereas a mass of
particles can take on any shape (such as uniform or bimodal).

Since the model of the car is nonlinear, a standard Kalman filter would not work,
and extended and unscented Kalman filters were considered in detail.
A basic EFK was constructed in MATLAB in [`ekf_demo.m`](scripts/ekf_demo.m).

A comparison of their performance can be seen by running [`compare_filters.m`](scripts/compare_filters.m)
in MATLAB. This generates a ground truth, samples noisy measurements from it,
then simulates an EKF, UKF, and particle filter on the data. The particle filter
was found to have superior performance, so it was implemented in Python for the
project.

## Controller
The controller is responsible for making the car do what you tell it to do.
Simply, that means you give it a [`ControlCommand.msg`](../rktl_msgs/msg/ControlCommand.msg)
and it produces a [`ControlEffort.msg`](../rktl_msgs/msg/ControlEffort.msg) that
makes the car follow it. At a basic level, it uses the odometry produced by the
filter and tries to minimize the error between what you want the car to do and
what it is actually doing.

Let's quickly discuss each control message in more detail:

**command**
- curvature and velocity
- fixed units (m^-1 and m/s)
- specific to a desired motion, not to a desired vehicle / car

**effort**
- steering and velocity
- range between -1.0 and +1.0
- specific to what is being controlled

The incoming message is different from the outgoing message because it is taking
a desired motion that could be for any mechanism, and it is converting it to
very specific hardware commands for our specific cars. This allows the software
generating the commands to ignore the specifics of the cars, and the software /
hardware getting the efforts to the actuators to also ignore the specifics of the
car as a whole.

To get slightly more complex, let's talk about how it actually does that. First,
it uses a bicycle model (which is discussed more in the particle filter) to
convert desired the velocity and curvature of the center of mass to a desired
steering angle and linear velocity of the rear wheels.

For steering, it calculates the actuator effort to make that happen and sends
it out. If it turns out to be off by a little bit, it ignores that. That is
called "open loop" since the information flows in a straight line and the actual
measurements are never used to correct that small error, "closing the loop." This
is OK since it probably won't be off by much, and whatever generates the command
will issue corrections (by closing its own loop) so the car goes where it is
supposed to. In the future, a closed loop controller could be implemented for
steering if testing determines it is necessary.

For velocity, it uses one of two controllers (PID or lead-lag) to calculate the
effort. These are more complex to understand, but they are basically functions
that will minimize error, and you can "tune" them to minimize the error in
different ways (ex: faster vs slower, more vs less overshoot). See
**Closed Loop Controllers** below for more information. To use them, the controller
will calculate an error between the desired (called "reference") rear wheel
velocity and the actual (from measurements). This is the "closing the loop" part.
It will then feed that error into the controller and it will make a good guess
as to what effort (for the rear motor) will minimize that error. For example, it
might initially give the car a really high effort to make it accelerate quickly,
then lower the effort once it reaches the desired speed. It might also see that
the car is consistently moving a little too slow, so it'll bump up the effort
more than originally predicted. *If the controllers are properly tuned*, this
will result in the car better matching what you want it to do as compared to
simply predicting an effort based off velocity.

### Closed Loop Controllers
The two types of controllers supported are PID and lead-lag. If you've never
heard of these before, PID is going to be something you can intuitively understand with some work,
and lead-lag is probably something you're going to need to take some classes or
read some textbooks to understand (ex: Purdue ME 365, 375, 475).

PID control is very well explained in many places on the internet, so I won't
discuss the basics here. The reason it is an option is because it is a commonly
used controller that can be tuned intuitively by watching how the car is behaving.
Lead-Lag is a very similar algorithm that can result in better performance
(especially for discrete systems like this), but it is harder to tune. Intuitively
tuning likely isn't an option, so instead MATLAB scripts can be really helpful
to determine the specific gains to use. These are included in `scripts`.

Some possibly useful references:
- <https://www.youtube.com/watch?v=UR0hOmjaHp0>
- <https://www.youtube.com/watch?v=wkfEZmsQqiA&list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y>
- <https://www.youtube.com/watch?v=xLhvil5sDcU>

These are all by Brian Douglas, who has many useful videos on control concepts.

## Hardware Interface
This takes a [`ControlEffort.msg`](../rktl_msgs/msg/ControlEffort.msg)
message and sends it to a physical car. The efforts in the message range from
-1.0 to +1.0 corresponding to full reverse for full forward motor speed,
or full left or full right steering angle respectively. Technically, you can
also feed in values with an absolute value greater than 1.0. This is useful for
having higher actuator saturation limits in the controller, or perhaps
implementing a "boost" for human controlled cars.

Physically, the interface code runs on a [Teensy 3.2](https://www.pjrc.com/store/teensy32.html)
microcontroller. The microcontroller is connected to a computer via USB, and
code running on the computer allows it to pull control effort messages from the
ROS network. The microcontroller is also connected to [FrSky XJT](https://www.frsky-rc.com/xjt/)
radio transmitter, which allows it to send these controls to the car.

To launch the node, simple give the Teensy power. To connect it to ROS on your
computer, launch the docker container with `./docker/docker-run.sh --privileged`
(this will only work on Linux), then launch the interface code with:
```
roslaunch rktl_control hardware_interface.launch
```

To use a different serial port (check `ls /dev` to see what it is named), modify
the config file.

You can enable the cars with:
```
rostopic pub /cars/enable std_msgs/Bool "data: true"
```

Then manually feed it controls with:
```
roslaunch rktl_control keyboard_control.launch
```

### Building and Uploading
Unfortunately, there are several steps required to build and upload code.

First you need to configure your IDE. This is best done *outside* of the Docker
container. First, install the Arduino IDE, then Teensyduino. Make sure to install
the PulsePosition library when installing Teensyduino.

Next, build the ROS libraries. *Inside the Docker container*, run:
```
rosrun rosserial_arduino make_libraries.py .
```
Copy the generated `ros_lib` folder to `<Arduino Location>/libraries/`

Lastly, launch the Arduino IDE, load the `hardware_interface.ino` project inside the
`scripts` folder, set the target board to the Teensy 3.2, and click upload.

### Further Information
`Teensyduino`, software that lets you program the Teensy using the Arduino IDE, is described here: <https://www.pjrc.com/teensy/teensyduino.html>

The specific library used to connect to the modules is described here: <https://www.pjrc.com/teensy/td_libs_PulsePosition.html>

Additional `rosserial_arduino` tutorials can be found here: <http://wiki.ros.org/rosserial_arduino/Tutorials>
