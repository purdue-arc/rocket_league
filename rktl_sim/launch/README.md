# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch rktl_sim <launch file>
```

:::{contents} Launch Files in the package
:depth: 2
:backlinks: top
:local: true
:::

---

## simulator.launch

This launch file launches a [simulator](../nodes/README.md#simulator) node and
sets the appropriate parameters. Parameters are set in
`rktl_sim/config/simulator.yaml` and in the launch file itself. For more
information, please refer to the documentation for the
[simulator](../nodes/README.md#simulator) node.

### Launch Arguments

- `pybullet_render` (default: `false`): Whether or not to render the pybullet simulator.
- `sim_mode` (default: `'realistic'`): Either set to `'realistic'` or `'ideal'`.
    See [simulator](../nodes/README.md#simulator) for more details.


### Nodes

- `simulator`: [`simulator`](../nodes/README.md#simulator) node from the
    [`rktl_sim`](../README.md) package.

---

## visualizer.launch

This launch file launches a [visualizer](../nodes/README.md#visualizer) node and
sets the appropriate parameters. Parameters are set in
`rktl_sim/config/visualizer.yaml` and in the launch file itself. For more
information, please refer to the documentation for the
[visualizer](../nodes/README.md#visualizer) node.

### Nodes

- `visualizer`: [`visualizer`](../nodes/README.md#visualizer) node from the
    [`rktl_sim`](../README.md) package.

---
