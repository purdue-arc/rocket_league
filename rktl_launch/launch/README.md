# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_launch <launch file>
```

:::{contents} Launch Files in the package
:depth: 2
:backlinks: top
:local: true
:::

---

## c3po.launch

This launch file includes two instances of the `camera.launch` file from the
[`rktl_perception`](../../rktl_perception/README.md) package with different
values for the `camera_name` argument. This is the launch file that will be run
on the computer named "c3po" on the Rocket League cart.

### Launch Arguments

This launch file does not define any arguments of its own.

### Nodes

This launch file does not define any nodes of its own.

### Included Launch Files

- `rktl_perception camera.launch` (Included twice, once with `camera_name` set
    to `cam0` and once with `camera_name` set to `cam1`)

---

## r2d2.launch

This launch file includes two instances of the `camera.launch` file from the
[`rktl_perception`](../../rktl_perception/README.md) package with different
values for the `camera_name` argument, launches a node from the
[`rktl_control`](../../rktl_control/README.md) package, and includes the
`rocket_league.launch` file from the [`rktl_launch`](../README.md) package.
This is the launch file that will be run on the computer named "r2d2" on the
Rocket League cart.

### Nodes

- `rktl_control pose_sync_node` (parameters are loaded from
    `rktl_control/config/pose_synchronizer.yaml`)

### Included Launch Files

- `rktl_perception camera.launch` (Included twice, once with `camera_name` set
    to `cam2` and once with `camera_name` set to `cam3`)
- `rktl_launch rocket_league.launch`

---

## rocket_league.launch

This roslaunch file launches everything that is needed to run the system using
actual hardware. It can be run with different types of agents and includes
launch files for the game manager, control GUI, ball, cars, and agents based on
the selected agent type.

### Launch Arguments

- `render` (default: `true`): A boolean argument that determines whether to
    enable the visualizer or not.
- `agent_type` (default: `patrol`): A string argument that specifies the type of
    agent to use. Valid options are `planner`, `autonomy`, or `patrol`.
- autonomy_weights (default: `model`): A string argument that specifies the name
    of the autonomy weights file to use. This argument is only used if
    `agent_type` is set to `'autonomy'`.

### Nodes

- `rqt_gui`: `rqt_gui` node from the `rqt_gui` package

### Included Launch Files

- `rktl_sim visualizer.launch` (Only included if `render` is set to `true`)
- `rktl_game game.launch`
- `rktl_control ball.launch`
- `rktl_control car.launch`
- `rktl_control hardware_interface.launch`
- `rktl_planner simple_agent.launch` (Only included if `agent_type` is set to
    `'planner'`)
- `rktl_autonomy rocket_league_agent.launch`: (Only included if `agent_type` is
    set to `'autonomy'`)
- `rktl_planner patrol_agent.launch`: (Only included if `agent_type` is set to
    `'patrol'`)

---

## rocket_league_sim.launch

This roslaunch file launches a simulation of the environment. It includes a
visualizer, a simulator, simulated perception delays, and control/filtering
stacks for the ball and cars. It also includes agents for controlling the cars,
with options for a planner, an autonomy (ML) agent, or a patrol agent.

### Parameters

This launch file loads parametes from the `config/global_params.yaml` file
in the [`rktl_launch`](../README.md) package.

### Launch Arguments

- `render` (default: `true`) - whether to launch the visualizer or not.
- `sim_mode` (default: `realistic`) - the simulation mode to use, either
    realistic or ideal.
- `perception_delay` (default: `0.15`) - the delay time to use for perception
    in the simulation.
- `agent_type` (default: `patrol`) - the type of agent to use for controlling
    the car, either `planner`, `autonomy`, or `patrol`.
- `autonomy_weights` (default: `model`) - the weights to use for the autonomy
    agent if agent_type is set to autonomy.

### Nodes

- `pose_sync_node`: A `pose_sync_node` node from the
    [`rktl_control`](../../rktl_control/README.md) package
- `ball/pose_delay`: A `topic_delay` node from the
    [`rktl_control`](../../rktl_control/README.md) package. Uses the
    `perception_delay` argument. Only run if `sim_mode` is set to `'realistic'`.
- `cars/car0/pose_delay`: A `topic_delay` node from the
    [`rktl_control`](../../rktl_control/README.md) package. Uses the
    `perception_delay` argument. Only run if `sim_mode` is set to `'realistic'`.

### Included Launch Files

- `rktl_sim visualizer.launch` (Only included if `render` is set to `true`)
- `rktl_sim simulator.launch` (Passes the `sim_mode` argument)
- `rktl_control ball.launch`
- `rktl_control car.launch`
- `rktl_planner simple_agent.launch` (Only included if `agent_type` is set to
    `'planner'`)
- `rktl_autonomy rocket_league_agent.launch`: (Only included if `agent_type` is
    set to `'autonomy'`)
- `rktl_planner patrol_agent.launch`: (Only included if `agent_type` is set to
    `'patrol'`)

---
