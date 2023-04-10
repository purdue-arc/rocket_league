# Launch Files

These are [.launch files](https://wiki.ros.org/roslaunch/XML) that can be run
using [roslaunch](https://wiki.ros.org/roslaunch):

```shell
roslaunch <Package Name> <launch file>
```

**Launch Files:**
- [`my_launch_file.launch`](#my-launch-file-launch)

---

## rocket_league.launch
This roslaunch file launches launches multiple nodes or inlcudes other launch
files, while posisbly also taking some arguments and setting some parameters.

### Launch Arguments
- `arg1` (default: `true`): A boolean argument that determines something.

### Nodes
- `node_name`: `foo` node from the `my_package` package
- `node_namespace/node_name`: `bar` node from the `my_package` package

### Included Launch Files
- `my_package foo.launch`
- `my_package bar.launch` (Only included if `arg1` is set to `true`)

---
