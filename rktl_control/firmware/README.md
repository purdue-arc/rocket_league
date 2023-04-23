# Microcontroller Firmware

This folder contains all firmware that is written to microcontollers for the
project.

```{contents} Firmwares in the package
:depth: 2
:backlinks: top
:local: true
```

---

## hardware_interface

This is the firmware that is written to a
[Teensy 3.2](https://www.pjrc.com/store/teensy32.html). The Teensy is connected
to a computer via USB, and code running on the computer allows it to pull
control effort messages from the ROS network. The Teensy is also connected to
[FrSky XJT](https://www.frsky-rc.com/xjt/) radio transmitter, which allows it
to send these controls to the car.

To launch the node, simply give the Teensy power and use the
[`hardware_interface`](../launch/README.md#hardware-interface-launch) launch
file:

```{shell}
roslaunch rktl_control hardware_interface.launch
```

Note that in order for this node to work, it need to be run on a Linux computer
with that the [docker container](../../docker/README.md) needs to be run with
the `--privileged` option.

To use a different serial port (check `ls /dev` to see what it is named), modify
the config file.

You can enable the cars with:

```{shell}
rostopic pub /cars/enable std_msgs/Bool "data: true"
```

Then manually feed it controls with:

```{shell}
roslaunch rktl_control keyboard_control.launch
```

### Building and Uploading

Unfortunately, there are several steps required to build and upload code.

First you need to configure your IDE. This is best done *outside* of the Docker
container. First, install the Arduino IDE, then Teensyduino. Make sure to install
the PulsePosition library when installing Teensyduino.

Next, build the ROS libraries. *Inside the Docker container*, run:

```{shell}
rosrun rosserial_arduino make_libraries.py .
```

Copy the generated `ros_lib` folder to `<Arduino Location>/libraries/`

Lastly, launch the Arduino IDE, load the `hardware_interface.ino` project inside the
`scripts` folder, set the target board to the Teensy 3.2, and click upload.

### Binding the Radios

Binding the radios to the cars is a somewhat tedious process. There is a very
precise set of instructions to follow, and you may need to do it several times
in a row, until everything goes exactly right.

1. Power everything off (radio + car)
2. Ensure the dip switches on the radio are set to 1-on, 2-off
3. Hold down the white button on the radio and power it. It will beep and blink
4. Hold down the button on the car's receiver, and power it. It will also blink
5. You can release both buttons
6. Power off the car
7. Power off the radio
8. Power on the radio
9. Power on the car
If it worked, the car's receiver will have a solid red light. If it didn't work,
you'll need to try again.

### Setting the Radio Failsafe

If the radio signal is lost, the car will automatically go in to a failsafe
mode. This mode needs to be programmed so that the default action is to do
nothing (as opposed to go absolutely bonkers and run into walls, which has a
good chance of happening if the failsafe is not explicitly set). The procedure
is simple. Run the hardware interface and manually publish an effort message
with the field's zeroed (do this on a frequency like 10 Hz). Power on the car
and within a second or two *after* power it, click the button on the receiver.
All done.

### Programming the ESCs

The ESCs (Electronic Speed Controllers) in the cars are very advanced. They're
over-powered for what we're doing, and are some of the best on the market. They
are highly configurable through a simple user interface.

Download and install
[Castle Link](https://home.castlecreations.com/download-castle-link/)
on a Windows computer. Use the
[USB adapter](https://www.castlecreations.com/en/pc-software-and-cables-4/castle-link-v3-usb-programming-kit-011-0119-00)
that we purchased with the ESCs to connect it to your computer. The battery does
not need to be connected to the car, and you will unplug the ESC from the
receiver to connect it to your computer. In Castle Link, connect to the car.
Then go to File->Load Settings, and load `esc_settings.dat` in the `config`
folder. Click Update in the bottom right to send them to the car.

After programming them, the endpoints need to be set. This tells it what
control input maps to what speed. The procedure is a little complex, but not too
bad.

1. Power off the car and hardware interface
2. Set the throttle throw to 500 on the hardware interface config
3. Launch and enable the hardware interface
4. Manually publish an effort message with 1.0 throttle effort (at 10 Hz)
5. Power on the car **With the wheels off the ground**, it should make a lot of beeping noises
6. Manually publish an effort message with -1.0 throttle effort (at 10 Hz)
7. Manually publish an effort message with 0.0 throttle effort (at 10 Hz)
8. Power off the car and hardware interface
9. Revert the throttle throw to what it was previously at

After step 7, the car should have made some noises, then stopped beeping. If you
run the car normally and it makes a lot of noises and doesn't move, or if it
goes *way* too fast, then you likely need to do this procedure again.

### Further Information

`Teensyduino`, software that lets you program the Teensy using the Arduino IDE, is described here: <https://www.pjrc.com/teensy/teensyduino.html>

The specific library used to connect to the modules is described here: <https://www.pjrc.com/teensy/td_libs_PulsePosition.html>

Additional `rosserial_arduino` tutorials can be found here: <http://wiki.ros.org/rosserial_arduino/Tutorials>

Radio transmitter manual: <https://www.frsky-rc.com/wp-content/uploads/2017/07/Manual/XJT.pdf>

Radio receiver manual: <https://cdn.shopify.com/s/files/1/0609/8324/7079/files/R84_Receiver_User_manual.pdf?v=1639375461>

ESC manual: <https://www.castlecreations.com/en/1-18th-scale/sidewinder-micro-2-esc-010-0150-00>

---
