# RacerSim

Allows virtual testing of pure-software components through simulating the physical environment and its localization.

## Constants

The simulator has multiple constants which can be found in the `YAML_files/sim_info.yaml` file. All non-abstract constants are in SI (metric) unless otherwise specified. E.g. metres, kilograms, and newtons. Some numbers have been measured, others are made up.

## ROS Interface

The `nodes/racersim_node` file interfaces with ROS. It does the following:

- Reads constants from the YAML file (`YAML_files/sim_info.yaml`)

- Subscribes to `car0/{velocity, path, lookahead_pnt}` which are sent by the Waypoint Controller

- Publishes to `{car0, ball}/odom`

- Initializes the Sim class (found in `sim.py`)

- Performs a sim step (with some Δt)

## Implementation Details

The `src/racersim/sim.py` is the most important file for the simulator. It contains the following.

- An initializer
- The step function

When the Sim class is initialized, it will also initialize the world (field implementation of Box2D's world), tires, cars, and ball. These objects are defined in an OOP-manner and can be found in `src/racersim`.

The initializer will also create either a random/semi-random/deterministic start position and angle for the ball and cars. The default is currently deterministic for testing purposes, this can be changed by commenting/uncommenting the respective lines.

The Sim class contains a step function to carry out a single update. This function will call 2 other step functions defined by the car and Box2D:

- The `self.car.step()` function calls the step function in `car.py`. This function updates the friction, wheel angles and speed through commands sent by the waypoint controller.

- The `self.world.Step()` function makes an internal call to Box2D updating the simulated environment. None of the previous updates will take effect unless this is called.

> Any new cars will need their `step()` function called above.

It will also call the `self._render()` function which draws every object in a new window. Rendering can be disabled in `sim_info.yaml`, its enabled by default.

If a window doesn't appear and you are using WSL2, check that you have properly configured the X-server.

**Common Mistakes**

The x-coordinates are intuitive. Larger x-coord ==> more to the right.

The y-coordinates are not intuitive. Larger y-coord ==> further down.

The coordinate (0,0) is found in the top-left of the window.

This causes a lot of confusion during angular velocity calculations.
