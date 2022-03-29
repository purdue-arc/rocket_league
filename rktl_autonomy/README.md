# Rocket League Autonomy
This package provides all code required to train a neural network to drive the
car and interface it into the existing control structure. It also provides
equivalent interfaces for two other environments (snake game and cartpole) for
verification purposes.

## Motivation
There are several tools out in the world that exist to make building, training,
and evaluating neural networks easier. However, for the most part, they need an
environment where you can fully control it from within the loop that is training
the network. In other words, time speeds up in the simulation when the loop runs
faster, time slows down when the loop runs slower, and time stops completely
when it pauses in order to back-propagate and calculate new network weights.
These environments also need discrete time steps, meaning you get an observation,
you provide an action, then time steps once, and it repeats.

This package manipulates a ROS network so that it provides that. In short, it
uses the sim time mechanism used to replay bag files to mess with how time
progresses, so that an arbitrary ROS network can be used as an environment,
specifically, an OpenAI Gym environment.

This can be used for both training and evaluation purposes. When training, it
manipulates time so that the network gets what it needs. It evaluation, it
runs the network as fast as real time is progressing.

This rather complex approach for training was used because the "environment"
originally included several different ROS nodes that would post-process the
output of the network before making a car move. It would have been really
difficult to remove ROS from the equation. Now, the training environment is
almost a direct link to a Python simulator, so ROS is mostly overhead during
training. We keep using it because it works and in the future we may train on
the more complex environment (which will require a more complex agent, perhaps
one with memory).

In the future, if it is desired to fully optimize speed when training on the
simplified environment, an alternative Gym environment wrapper should be created
that directly owns a Simulator object provided in `rktl_sim`. This will remove
the ROS middle-man and likely provide a small boost in training speed.

## Environments
The way the package is set up, there is a class, `ROSInterface`, that handles
the base behavior of manipulating time as discussed above. It gets subclassed
to customize it to a specific ROS network's environment. Three such environments
are provided.

### Rocket League
This the main environment intended for use. It interfaces the agent with the
mess of other nodes that exist in this repository. Look at higher level
documentation to learn more about the inputs and outputs from this node and how
they are consumed or created by others.

### Snake Game
This provides an interface to the snake game created in the [ARC Tutorials](https://github.com/purdue-arc/arc_tutorials/tree/snake_dqn).
This was largely to gain experience in training reinforcement learning agents
and there isn't much need for it anymore. It may be useful for testing new types
of agents or onboarding new members.

### Cartpole
Cartpole is a very simple environment included within OpenAI Gym. There is not
much of a reason to use it, except for verifying the functionality of this
library. Two different interfaces are provided, `CartpoleInterface` and
`CartpoleDirectInterface`. The former uses the `ROSInterface` class and the latter
directly owns the Gym environment. To verify the `ROSInterface` worked, it was
confirmed that they both behave the same way.

## Training
Training is the process for determining the weights used in the agent's neural
network. As discussed earlier, it needs to mess with sim-time, so there is a
specific way to launch it. **You should not manually use any ROS commands when
launching for training.**

### Training Script
There are scripts for each environment listed above. This will describe the
Rocket League one, but the others are very similar.

This script basically does everything to get an agent training. It will generate
a UUID (used for logging weights / progress), build the environment(s) (which
will internally handle everything with ROS), set it up to periodically log, then
kick it off in a training loop that will run for a very long time.

If you want to modify the type of agent or hyperparameters, this is the place to
do it. There is no reason there can't be several different scripts to train
several different types of agents.

#### Implementation details
When you look under the hood into how `ROSInterface` launches the different
ROS networks, things get a little messy. This will explain how that works,
mainly so that it can be fixed if it ever breaks.

When you instantiate a `ROSInterface` in training mode, it launches the entire
ROS network needed for training, then makes itself a node in that network. This
is done by giving it a launch file. These are appended with `_train` in the
launch file directory in this package. If you want to modify the environment
from a ROS perspective, edit that. From the training script, you can pass in
ROS arguments (like on the command line), so you can have different options
exposed that way.

Things get messy when you want to use vectorized environments to run several
environments at once for a single agent. This increases the amount of data it
receives per time-step and can increase the training speed.

Therefore, running vectorized environments means you need multiple ROS networks
running at once (since each entire network is an environment). This is OK since
ROS can run on ports other than the default of 11311.

Each physical process on the computer can only be one ROS node in one ROS
network. Therefore, the training script itself needs to spawn multiple
subprocesses for each environment. This is easily done through StableBaselines3's
`SubprocVecEnv` class.

These separate processes now need to each get their own unique port to run a ROS
network on. This is done through negotiation with temporary files. One
environment will end up with the default port of 11311 (useful so your ROS
commands work without needing to change `ROS_MASTER_URI`) and the others will be
scattered on random free ports. To prevent deadlocks, it is configured to launch
only one environment at a time. This can be a little slow, but it is robust.

An issue arises when you want to terminate training early. There might be a bug
in `SubprocVecEnv` or in our own code, but when you kill the training early with
`CTRL+C`, there are sometimes processes left hanging around that shouldn't be
there. You can kill these using terminal commands, or you can just kill the
Docker container and start a new one.

If different environments are run in different containers, negotiating ports is
not an issue since each container has its own virtual network.

#### Checking in on training
Since the environment is ultimately a ROS network, you can use all your regular
ROS commands to see what is happening. If you have a terminal into the Docker
container, you can run the visualizer: `roslaunch rktl_sim visualizer.launch`.

Keep in mind that the timescale of the simulation window will vary according to
CPU load for reasons discussed above. You can safely close the visualizer and
launch it again later.

The environment also has a node that generates a plot of performance over time.
These save into a logging folder `catkin_ws/data/rocket_league/<UUID>` by
default. There will typically be one plot, `plot_11311.png` by default. If using
vectorized environments, you can make it produce a plot for each environment if
desired, but they should all have equivalent behavior.

The StableBaselines3 library is also configured to output data on the training
progress, which saves to a text and csv file in that same folder. That is
sometimes less helpful to look at since it is harder to interpret.

There will also be many zip files in the same folder. These are the saved model
weights, which can be loaded in order to run the agent in evaluation mode later.
See below.

#### Stopping training
Simply `CTRL+C` the training script. Since it has to run for a long period of
time to train, you probably want to use `tmux`. As noted in **Implementation details**
above, it might leave a mess you need to clean up.

After stopping training, all the log files noted in the above section will still
exist, and you can even resume training using the zip files if you had to. This
isn't implemented in the training script now, but it should be possible.

### Batch Train Script
This script exists to simplify the process of checking out the proper code,
launching a Docker container, and running the above training script when many
different experiments are intended to be run simultaneously. To use, simply
note the commit numbers of the configurations you want to run (`git log` may be
useful), then run the following command on the machine:

```
./rktl_autonomy/scripts/train_batch.sh <list of git commit hashes>
```

#### Detailed Git instructions
- On your computer, `git checkout` the branch you want to use as a starting point.
    This might be `main` or a testing branch you already have.
- If desired, create a new branch for your changes: `git checkout -b <new branch name>`
- Modify your files locally to configure it for the experiment you want to run
- If desired, run it briefly to make sure it doesn't crash using the above basic
    training script. Make sure to reduce the number of parallel environments to
    1, to not overly strain your computer. Remember to set it back if you modify
    it and want it to run with several (24 max recommended) on the supercomputer.
- When satisfied, run `git commit -am <message describing the experiment>`.
- Repeat the above 2 steps as many times as desired for all the experiments you
    want to run simultaneously.
- Push your changes to GitHub using `git push`. If you made a new branch, it'll
    prompt you with an exact command you can copy-paste to push the new branch.
- Run `git log` to see the commit hashes of the commits you pushed.
- Access the supercomputer and navigate to the catkin workspace (no need to
    launch the Docker container)
- Run `docker container ls` to see how many jobs are currently running. The max
    is around 7, but you can use `htop` to see the real-time load on the computer
    and decide if it can handle another.
- Run `git fetch` so Git is aware of your changes that you just pushed
- Run `git checkout main` if you were on another branch, so that you are using
    the latest stable version of the script.
- Run the script using the above command and all the git commit hashes you want
    to test. Separate them by spaces.
- Copy-paste the output to some place you'll remember.

#### Checking in on training
You can use all of the methods discussed in the regular training script to check
on progress. The only difference is that you need to know where to find the
files and how to connect to the Docker container.

To find the files, you can do one of two things. Either look at the output from
when you ran the batch train script and look for the run ID of each specific
experiment. You'll find these logs in the usual spot.

Alternatively, you can go to the log directory listed at the very top of the
batch script output. It'll be something like: `catkin_ws/data/rocket_league/batch_logs/<UUID>`.
Inside there, there will be symlinks (shortcuts) to the logging folders for each
experiment being run, named by the first 7 digits of the git commit has. Note
that the UUID for the experiment and for the batch script will be different.

To connect to the Docker container, again, there are a few different ways. You
must look at the output of the script to see what name in gave the container.
The container name will the the batch script's UUID, then the first seven digits
of the git commit hash, separated by a hyphen.

You can also run `docker container ls` and look at the container names and IDs.
You may be able to guess which line is your container given the naming scheme
and seeing how long it has been running. You can either use the ID or name. The
ID will be shorter but the name follows the naming convention noted in the above
paragraph.

Then you can attach your terminal to it with the following command:
```
docker exec -it <name or id> zsh
```

You can also use the join script if you want:
```
CONTAINER_NAME=<name or id> ./docker/docker-join.sh
```

#### Stopping training
Since the batch script runs all these containers in the background then exits,
`CTRL+C` won't do anything. Get either the container name or ID using methods
discussed above, then use the following command to stop it:
```
docker container kill <name or id>
```

### Hyperparameter Searching
TODO

## Real-Time Evaluation
At this point, you should have trained your network and have a zip file with
model weights. To evaluate, you can simply run the launch file for the
evaluation ROS network:
```
roslaunch rktl_autonomy rocket_league_eval.launch
```

You should rename the zip file as `catkin_ws/data/rocket_league/model.zip` or
use a command line argument to tell it to use another location:
```
roslaunch rktl_autonomy rocket_league_eval.launch weights:=<path and name minus '.zip'>
```
