# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch <Package Name> <launch file>
```

```{contents} Launch Files in the package
:depth: 2
:backlinks: top
:local: true
```

---

## rocket_league.launch

This roslaunch file launches launches multiple nodes or includes other launch
files, while possibly also taking some arguments and setting some parameters.

### Launch Arguments

- `arg1` (default: true): A boolean argument that determines something.

### Nodes

- `node_name`: `foo` node from the `my_package` package
- `node_namespace/node_name`: `bar` node from the `my_package` package

### Included Launch Files

- `my_package foo.launch`
- `my_package bar.launch` (Only included if `arg1` is set to `true`)

---
