# Rocket League
This repo contains all the files necessary for the [Rocket League project](https://wiki.purduearc.com/wiki/rocket-league/overview).
This project aims to recreate the video game of the same name via a team of
autonomously controlled RC cars competing against human controlled ones.

The repo is broken down into several ROS packages, closely matching the system outline:

![System outline](https://wiki.purduearc.com/wiki/rocket-league/assets/images/system-overview.png)

### rktl_autonomy
This package contains all code for the "High Level Planner" section. It provides
an interface between ROS and OpenAI Gym, and is therefore capable of running a
deep reinforcement learning agent capable of controlling the car.

For further information see its specific [`README.md`](rktl_autonomy/README.md) file.

### rktl_planner
This package contains the "Mid Level Software" group. It provides an alternative
method to the `rktl_autonomy` stack to control the car.

For further information see its specific [`README.md`](rktl_planner/README.md) file.

### rktl_control
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

For further information see its specific [`README.md`](rktl_control/README.md) file.

### rktl_perception
This package contains the "Perception" code. This interfaces with cameras, processes
the data, and outputs position estimates for all physical elements on the field.

For further information see its specific [`README.md`](rktl_perception/README.md) file.

### rktl_sim
This contains a simulator, which is used to train the autonomy code.

For further information see its specific [`README.md`](rktl_sim/README.md) file.

### rktl_msgs
This contains custom ROS messages for the project.

### rktl_launch
This contains several convenience launch files for running several packages at once.

## Additional Folders / Files
### docker
This contains scripts for building and running the Docker environment used for the project.
For more information, see the below **Building and Running the Project** section.

### .github
This contains GitHub actions used for continuous integration. Primarily, it automates:
- building the development Docker container
- running automated tests on PRs and pushes to main

### start.py
This contains a Python script used to launch the production system on the computer
cart. It launches Docker containers on multiple computers (defined in `hosts.yaml`),
and launches the necessary ROS nodes inside of them.

## Building and Running the Project
### Pre-Requisites
This project is built on [ROS (Robot Operating System)](https://www.ros.org/)
You should be familiar with using ROS on Linux inside Docker through previous
experience, or by following our super cool [tutorials](https://wiki.purduearc.com/wiki/tutorials/ros).

The project runs inside a development Docker container, so the host machine only
needs to be able to run Docker. All software pre-requisites are installed in the
image.

### Building
First, make sure your code is in the proper place. There is a specific directory
structure required, which is:
```
catkin_ws
 -> src
     -> rocket_league (this repo)
```

Starting from scratch in the parent directory for `catkin_ws`, run:
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone git@github.com:purdue-arc/rocket_league.git
```
> You should have a different catkin workspace for each project, so if you already
use ROS for something else (other than the ARC tutorials), you should name the
workspace something different such as `arc_ws` or `rktl_ws`.

Run all future commands inside the `rocket_league` folder.

First, pull a mostly built Docker container, and customize it for your machine:
```
./docker/docker-build.sh
```

Next, launch the Docker container:
```
./docker/docker-run.sh
```
Your terminal will be transported into the container. The directory `~/catkin_ws`
inside the container is mapped to the workspace wherever you put it, and whatever
you named it outside the container. Changes in here are permanent when the container
exist, but installing software and other actions are all isolated from your main
computer and will be lost when you exit the container.

Finally, build the code:
```
cd catkin_ws
catkin build
```

Optionally, run the automated tests:
```
catkin test
```

### Running
> This resumes where **Building** left off, so run all these commands in the
container, not your host computer

To run the project's simulator and visualizer, run:
```
roslaunch rktl_launch rocket_league_sim.launch
```

To manually give the car input, run:
```
roslaunch rktl_control keyboard_control.launch
```
> This must be done while the first command is still running. You can either:
> - run `./docker/docker-join.sh` in another terminal on your host machine to
transport that terminal into the same container
> - use `CTRL+Z` and `bg` to push the current process into the background, so that
you can run another command. (use `fg` to bring it back into the foreground)
> - use [`tmux`](https://tmuxcheatsheet.com/) to have multiple terminals inside one

To stop the current process, type `CTRL+C`.
To exit the container, simply type `exit` or `CTRL+D`.
