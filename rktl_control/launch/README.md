# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_control <launch file>
```

**Launch Files:**

- [`ball.launch`](#ball-launch)
- [`car.launch`](#car-launch)
- [`hardware_interface.launch`](#hardware-interface-launch)
- [`keyboard_control.launch`](#keyboard-control-launch)
- [`xbox_control.launch`](#xbox-control-launch)

---

## ball.launch

This launches a [`mean_odom_filter`](../nodes/README.md#mean-odom-filter) node
to start the ball tracking with parameters supplied by
`rktl_control/config/mean_odom_filter.yaml`.

### Nodes

- `ball/mean_odom_filter`:
    [`mean_odom_filter`](../nodes/README.md#mean-odom-filter) node from the
    [`rktl_control`](../README.md) package. Parameters supplied by
    `rktl_control/config/mean_odom_filter.yaml`.

---

## car.launch

This launches the filter (either a
[particle filter](../nodes/README.md#particle-odom-filter) or a
[rolling average](../nodes/README.md#mean-odom-filter)) to track the car, as
well as the [controller](../nodes/README.md#controller) needed to control it.
The type of filter can be changed by the `use_particle_filter` argument.
Parameters are provided by `rktl_control/config/particle_odom_filter.yaml` or
`rktl_control/config/mean_odom_filter.yaml` (depending on the filter chosen) and
`rktl_control/config/controller.yaml`.

### Launch Arguments

- `car_name` (default: `car0`): Name of the car to launch the agent for.
- `use_particle_filter` (default: true): If true, a
    [particle filter](../nodes/README.md#particle-odom-filter) is used.
    Otherwise, a [rolling average](../nodes/README.md#mean-odom-filter) is used.

### Nodes

- `mean_odom_filter`:
    [`mean_odom_filter`](../nodes/README.md#mean-odom-filter) node from the
    [`rktl_control`](../README.md) package. Only run if `use_particle_filter` is set to false. Parameters supplied by
    `rktl_control/config/mean_odom_filter.yaml`.
- `particle_odom_filter`:
    [`particle_odom_filter`](../nodes/README.md#particle-odom-filter) node from the
    [`rktl_control`](../README.md) package. Only run if `use_particle_filter` is set to true (the default). Parameters supplied by
    `rktl_control/config/particle_odom_filter.yaml`.
- `controller`:  [`controller`](../nodes/README.md#controller) node from the
    [`rktl_control`](../README.md) package. Parameters supplied by
    `rktl_control/config/controller.yaml`.

---

## hardware_interface.launch

This takes a [rktl_msgs/ControlEffort](/rktl_msgs/html/msg/ControlEffort.html)
message and sends it to a physical car. The efforts in the message range from
-1.0 to +1.0 corresponding to full reverse for full forward motor speed, or
full left or full right steering angle respectively. Technically, you can also
feed in values with an absolute value greater than 1.0. This is useful for
having higher actuator saturation limits in the controller, or perhaps
implementing a "boost" for human controlled cars.

Physically, the interface code runs on a
[Teensy 3.2](https://www.pjrc.com/store/teensy32.html) microcontroller. The
microcontroller is connected to a computer via USB, and code running on the
computer allows it to pull control effort messages from the ROS network. The
microcontroller is also connected to [FrSky XJT](https://www.frsky-rc.com/xjt/)
radio transmitter, which allows it to send these controls to the car.

Note that in order for this node to work, it need to be run on a Linux computer
with that the [docker container](../../docker/README.md) needs to be run with
the `--privileged` option.

### Nodes

- `hardware_interface`: `serial_node.py` node from the
    [`rosserial_arduino`](https://wiki.ros.org/rosserial_arduino) package. This
    node allows the firmware on a connected Teensy to interface with the rest
    of the ROS network. Parameters, including the serial port used by the
    Teensy, are supplied by `rktl_control/config/hardware_interface.yaml`.

---

## keyboard_control.launch

This launches a [`keyboard_interface`](../nodes/README.md#keyboard-interface)
node to allow for controlling the car using the keyboard.

### Nodes

- `cars/{car_name}/keyboard_interface`:
    [`keyboard_interface`](../nodes/README.md#keyboard-interface) node from the
    [`rktl_control`](../README.md) package. The namespace of this node is
    affected by the `car_name` parameter to ensure that the node interacts with
    the correct topics.

### Parameters

- `car_name` (default: `car0`): Name of the car to control with the keyboard
    interface.

---

## xbox_control.launch

This launches a [`xbox_interface`](../nodes/README.md#xbox-interface)
node and all related nodes to allow for controlling the car using an xbox
controller.

### Nodes

- `cars/{car_name}/joy_node`:
    [`joy_node`](https://wiki.ros.org/joy#joy_node) node from the
    [`joy`](https://wiki.ros.org/joy) package. This node publishes the current
    state of the joystick to be used by the
    [`xbox_interface`](../nodes/README.md#xbox-interface) node. The namespace
    of this node is affected by the `car_name` parameter to ensure that the
    node interacts with the correct topics.
- `cars/{car_name}/xbox_interface`:
    [`xbox_interface`](../nodes/README.md#xbox-interface) node from the
    [`rktl_control`](../README.md) package. The namespace of this node is
    affected by the `car_name` parameter to ensure that the node interacts with
    the correct topics.
- `cars/{car_name}/controller_delay`:
    [`topic_delay`](../nodes/README.md#topic-delay) node from the
    [`rktl_control`](../README.md) package. This adds delay to the output of
    the `joy_node` topic The namespace of this node is affected by the
    `car_name` parameter to ensure that the node interacts with
    the correct topics.
- `cars/{car_name}/controller_delay_mux`:
    [`mux`](https://wiki.ros.org/topic_tools/mux) node from the
    [`topic_tools`](https://wiki.ros.org/topic_tools) package. This node
    subscribes to the raw joystick and delayed joystick topics and republishes
    it onto a combined topic. The namespace of this node is affected by the
    `car_name` parameter to ensure that the node interacts with the correct
    topics.

### Parameters

- `device` (default: `/dev/input/js0`): Device file of the joystick.
- `car_name` (default: `car0`): Name of the car to control with the keyboard
    interface.
- `delay` (default: `0.1`): Amount of simulated delay to add to the controller
    input

---
