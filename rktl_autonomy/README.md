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
This the main environment intended for use.

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
specific way to launch it. You should not manually use any ROS commands when
launching for training.

### Training Script

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


### Hyperparameter Searching
TODO

## Real-Time Evaluation
