# Rocket League
This repo contains all the files necesary for the [Rocket League project](https://wiki.purduearc.com/wiki/rocket-league/overview). This project aims to recreate
the video game of the same name via a team of autonomously controlled RC cars competing against human controlled ones.

The repo is broken down into several ROS packages, closely matching the system outline:

![System outline](https://wiki.purduearc.com/wiki/rocket-league/assets/images/system-overview.png)
### rktl_autonomy
This package contains all code for the "High Level Planner" section. It provides an interface between ROS and OpenAI Gym, and is therefore capable of running a deep reinforcement learning agent capable of controlling the car.

### rktl_planner
This package contains the "Mid Level Software" group. Currently, it is not in use, as the autonomy code is directly producing a velocity, but it may be reincorporated in the future.

### rktl_control
This package contains the "Velocity Controller" and "Hardware Interface" code. The hardware interface is an Arduino sketch included in the `scripts` directory. The velocity controller is a closed loop feedback controller used to keep the car's hardware close to the commanded reference.

### rktl_perception
This package contains the "Perception" code. This interfaces with cameras, processes the data, and outputs filtered position estimates for all physical elements on the field.

## Additional Folders / Packages
### docker
This contains scripts for building and running the Docker environment used for the project.

### rktl_test
This contains the automated tests for the project.

### rktl_sim
This contains a simulator, which is used to train the autonomy code.

### rktl_msgs
This contains custom ROS messages for the project.

### rktl_launch
This contains several convenience launch files for running several packages at once.


# Pre-Requisites
TODO

# Building
TODO

# Running
TODO
