# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_planner <launch file>
```

**Launch Files:**
- [`patrol_agent.launch`](#patrol-agent-launch)
- [`simple_agent.launch`](#simple-agent-launch)

---

## patrol_agent.launch
This launches a [`patrol_planner`](../nodes/README.md#patrol-planner) node with
parameters supplied by `rktl_planner/config/patrol_planner.yaml`.

### Launch Arguments
- `car_name` (default: `car0`): Name of the car to launch the agent for.

### Nodes
- `cars/{car_name}/patrol_planner`:
    [`patrol_planner`](../nodes/README.md#patrol-planner) node from the
    [`rktl_planner`](../README.md) package. Parameters supplied by
    `rktl_planner/config/patrol_planner.yaml`.

---

## simple_agent.launch
This launches nodes to generate and follow paths using bezier curve-based
methods. It loads parameters from `rktl_planner/config/path_follower.yaml` and
`rktl_planner/config/path_planner.yaml`.

### Launch Arguments
- `agent_name` (default: `agent0`): Name of the agent to run.
- `car_name` (default: `car0`): Name of the car to launch the agent for.

### Nodes
- `agents/{agent_name}/path_follower`:
    [`path_follower`](../nodes/README.md#path-follower) node from the
    [`rktl_planner`](../README.md) package. Parameters supplied by
    `rktl_planner/config/path_follower.yaml`.
- `agents/{agent_name}/bezier_path_server`:
    [`bezier_path_server`](../nodes/README.md#bezier-path-server) node from the
    [`rktl_planner`](../README.md) package.
- `agents/{agent_name}/path_planner`:
    [`path_planner`](../nodes/README.md#path-planner) node from the
    [`rktl_planner`](../README.md) package. Parameters supplied by
    `rktl_planner/config/path_planner.yaml`.

---
