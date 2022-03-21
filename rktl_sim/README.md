# Rocket League Sim

Allows virtual testing of pure-software components through simulating the physical environment and its localization.

## Constants

TODO

## ROS Interface

The simulation package has two launch files:
1. `simulator.launch`: Deploys virtualized field environment
2. `visualizer.launch`: Opens a window for viewing virtualized environment

When launch `simulator.launch`, there are a few arguments you should be aware of:
- `sim_mode`: This a string with two options: 'realistic' (default) or 'ideal'. 

    When using 'realistic' mode, an external filter is required as pose data is only being published to `pose_sync` and artificial noise may be set in the configuration file. Additionally the car will only read commands from the `effort` topic, which is control effort data.

    When using 'ideal' mode, the sim publishes data, without noise, to `odom` topics and forces the car to perfectly follow commands in the `command` topic.

## Implementation Details

TODO

**Common Mistakes**

TODO
