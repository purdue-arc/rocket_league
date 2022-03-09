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

### Particle Filter


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
