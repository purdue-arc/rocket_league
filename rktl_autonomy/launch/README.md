# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_autonomy <launch file>
```

:::{contents} Launch Files in the package
:depth: 2
:backlinks: top
:local: true
:::

---

## cartpole_train.launch

This launch file starts the training a model for the cartpole environment. Wraps
the [`cartpole_env`](../nodes/README.md#cartpole-env) node. For more info,
refer to its documentation.

### Launch Arguments

- `agent_name` (default: `rocket_league_agent`): Name of the agent.
- `render` (default: `false`): If true, a window for the current simulation will
    be shown.
- `rate` (value: `30.0`): Rate at which the simulation runs, in steps per second.

### Nodes

- `cartpole_env`: [`cartpole_env`](../nodes/README.md#cartpole-env) node in
    the [`rktl_autonomy`](../README.md) package.

---

## rocket_league_agent.launch

This launch file starts the agent for the rocket league environment. Wraps
the [`rocket_league_agent`](../nodes/README.md#rocket-league-agent) node. For
more info, refer to its documentation.

### Launch Arguments

- `plot_log` (default: `false`): If true, a plot will be generated.
- `weights_dir` (default: `'~/catkin_ws/data/rocket_league/'`): Folder
    containing the weights of pre-trained models.
- `weights_name` (default: `'model'`): Which of the models to load, from the
    directory defined in `weights_dir`.

### Nodes

- `rocket_league_agent`:
    [`rocket_league_agent`](../nodes/README.md#rocket-league-agent) node from
    the [`rktl_autonomy`](../README.md) package.
- `plotter`: [`plotter`](../nodes/README.md#plotter) node from the
    [`rktl_autonomy`](../README.md) package. Only run if the `plot_log` argument
    is set to `true`.

---

## rocket_league_train.launch

This launch file starts the training a model for the rocket league environment.
Wraps the [`rocket_league_agent`](../nodes/README.md#rocket-league-agent) node
and runs the simulator.

Loads a training environment for and starts training a model.

### Launch Arguments

- `plot_log` (default: `true`): If true, a plot will be generated.
- `agent_name` (default: `rocket_league_agent`): Name of the agent.
- `render` (default: `false`): If true, a window for the current simulation will
    be shown.
- `sim_mode` (default: `'ideal'`): Simulation mode for training.
- `rate` (value: `10.0`): Rate at which the simulation runs, in steps per
    second. Cannot be overridden.
- `agent_type` (value: `'none'`): Disables the loading of an existing agent.
    Cannot be overridden.

### Nodes

- `plotter`: [`plotter`](../nodes/README.md#plotter) node from the
    [`rktl_autonomy`](../README.md) package. Only run if the `plot_log` argument
    is set to `true`.

### Included Launch Files

- `rktl_perception camera.launch` (Included twice, once with `camera_name` set
    to `cam2` and once with `camera_name` set to `cam3`)
- `rktl_launch rocket_league_sim.launch`: Loads Parameters from
    `config/rocket_league.yaml` in the [`rktl_autonomy`](../README.md) package.
    Also sets `~log` to `$(arg agent_name)/log`.

---

## snake_eval.launch

This launch file starts the agent for the snake environment. Wraps the
[`snake_agent`](../nodes/README.md#snake-agent) node. For more info, refer to
its documentation.

### Launch Arguments

- `plot_log` (default: `false`): If true, a plot will be generated.
- `render` (default: `true`): If true, a window for the current simulation will
    be shown.
- `weights` (default: `'~/catkin_ws/data/snake/weights`'): Folder
    containing the weights of pre-trained models.
- `rate` (value: `10.0`): Rate at which the simulation runs, in steps per
    second. Cannot be overridden.
- `snake_size` (value: `7`): Initial number of segments for the snake. Cannot be
    overridden.
- `arena_size` (value: `10`): Size of the arena the snake operates in. Cannot be
    overridden.

### Nodes

- `snake_env`: `snake_node` node from the `snakesim` package.
- `snake_agent`: [`snake_agent`](../nodes/README.md#snake-agent) node from the
    [`rktl_autonomy`](../README.md) package.
- `plotter`: [`plotter`](../nodes/README.md#plotter) node from the
    [`rktl_autonomy`](../README.md) package. Only run if the `plot_log` argument
    is set to `true`.

---

## snake_train.launch

This launch file starts the training a model for the snake environment. Wraps
the [`snake_agent`](../nodes/README.md#snake-agent) node. For more info,
refer to its documentation.


### Launch Arguments

- `plot_log` (default: `false`): If true, a plot will be generated.
- `agent_name` (default: `'snake_agent'`):  Name of the agent.
- `render` (default: `false`): If true, a window for the current simulation will
    be shown.
- `rate` (value: `10.0`): Rate at which the simulation runs, in steps per
    second. Cannot be overridden.
- `snake_size` (value: `7`): Initial number of segments for the snake. Cannot be
    overridden.
- `arena_size` (value: `10`): Size of the arena the snake operates in. Cannot be
    overridden.

### Nodes

- `snake_env`: `snake_node` node from the `snakesim` package.
- `plotter`: [`plotter`](../nodes/README.md#plotter) node from the
    [`rktl_autonomy`](../README.md) package. Only run if the `plot_log` argument
    is set to `true`.

---
