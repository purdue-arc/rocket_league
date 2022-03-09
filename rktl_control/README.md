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
random ones.

The specific state of the car is tracked by five variables:
- x, the x location of the car's center of mass in the field's reference frame
- y, the y location of the car's center of mass in the field's reference frame
- theta, the car's heading angle in the field's reference frame
- v_rear, the linear velocity of the rear tires of the car
- psi, the steering angle (relative to the car's center)

From these, a kinematic bicycle model is used to calculate out everything needed
for an odometry message. Basically, this just assumes there is no side-slip by
the tires and it uses a lot of trigonometry. Details are in the MATLAB script
[`bicycle_model.m`](scripts/bicycle_model.m).

When advancing each particle one time-step, a random control vector (steering
and velocity efforts are produced). These are uniformly distributed from +/- 1.0 


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
TODO

## Hardware Interface
This takes a [`ControlEffort.msg`](../rktl_msgs/msg/ControlEffort.msg) 
message and sends it to a physical car. The efforts in the message range from
-1.0 to +1.0 corresponding to full reverse for full forward motor speed,
or full left or full right steering angle respectively.

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

To use a different serial port (check `ls /dev` to see what it is named), launch
with the following command:
```
roslaunch rktl_control hardware_interface.launch serial_port:=<port>
```

You can then manually feed it controls with:
```
roslaunch rktl_control keyboard_control.launch
```

### Known bugs
For some reason, you need to launch the computer side hardware interface code
twice. The first time, the car will stop moving, but steer fully to one side.
The second time, it will send the efforts from ROS to the car.

This isn't a bug necessarily, but it is odd. The parameters are loaded when the
node launches. The node launches when the device is powered. If you change the
parameters and relaunch the computer side hardware interface, they won't be
loaded in, because the node didn't re-launch. To fix this, power cycle the
board after changing parameters.

Perhaps this can be improved in the future using `dynamic reconfigure`.

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

In the future, perhaps this can be improved: <http://wiki.ros.org/rosserial_arduino/Tutorials/CMake>

### Further Information
`Teensyduino`, software that lets you program the Teensy using the Arduino IDE, is described here: <https://www.pjrc.com/teensy/teensyduino.html>

The specific library used to connect to the modules is described here: <https://www.pjrc.com/teensy/td_libs_PulsePosition.html>

Additional `rosserial_arduino` tutorials can be found here: <http://wiki.ros.org/rosserial_arduino/Tutorials>
