# Training Scripts

These are python scripts used to train the neural network. Each script basically
does everything to get an agent training. It will generate a UUID (used for
logging weights / progress), build the environment(s) (which will internally
handle everything with ROS), set it up to periodically log, then kick it off in
a training loop that will run for a very long time.

If you want to modify the type of agent or hyperparameters, this is the place to
do it. There is no reason there can't be several different scripts to train
several different types of agents.

**You should not manually use any ROS commands when launching for training.**

:::{contents} Training scripts in the package
:depth: 2
:backlinks: top
:local: true
:::

---

## eval_rocket_league.py

If you want to evaluate a bunch of models at once, to produce a plot of
performance over time (perhaps with a different environment or reward set from
what it was trained on), you can use the this convenience script:

```shell
# in Docker, ~/catkin_ws/src/rocket_league/rktl_autonomy/scripts
./eval_rocket_league.py <UUID of run>
```

`UUID of run` should be the name of a folder in the
`~/catkin_ws/data/rocket_league` directory containing all the weights you
with to evaluate. You can pass in multiple UUIDs separated by spaces, and it
will run the script multiple times.

For each UUID passed in, the script will generate a file in the same folder
titled `eval_log_<new UUID>.txt`. This is a tab-delimited file, which you can
open in Matlab or Excel to produce a plot.

---

## train_batch.sh

This script exists to simplify the process of checking out the proper code,
launching a Docker container, and running the above training script when many
different experiments are intended to be run simultaneously. To use, simply
note the commit numbers of the configurations you want to run (`git log` may be
useful), then run the following command on the machine:

```bash
./rktl_autonomy/scripts/train_batch.sh <list of git commit hashes>
```

---

## train_cartpole.py

Training script for the `CartPoleInterface` for testing.

---

## train_rocket_league.py

---

Training script for the Rocket League project.

---

## train_snake.py

Training script for the Snake game in ARC tutorials.

---

## tune_rocket_league.py

This script will take the current game settings (rewards, episode length,
simulation parameters) as is, and only experiment with hyperparameters of the
PPO model.

These hyperparameters will be fed into the PPO initialization function as
`model_params` at the start of each tuning attempt. The choice of
hyperparameters is determined by Optuna based on the ranges given in the 
`optimize_ppo2()` function`. All of the chosen hyperparameter ranges can be
modified directly in the function's code.

Within the script, there are also 4 tuning variables that can be modified to
change the rate at which the tuning and training-batches occur.

The tuning script will output progress in the command window it was executed in.
Every 5 tuning attempts, it will print out the best hyperparameters that it
has found so far. These can later be copied and used in the
`train_rocket_league.py` script.

---
