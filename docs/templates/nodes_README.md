# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun <Package_name> <Node name>
```

**Nodes:**
- [`node_name`](#node-name)

---

## node_name
This node tracks does something that you should explain here.

### Subscribed Topics:
- `topic_foo` (my_pkg/MyMsg.msg): a topic that this node subscribes to

### Published Topics:
- `topic_bar` (my_pkg/MyMsg.msg): a topic that this node publishes to

### Services:
- `service_baz` (my_pkg/MySrv.srv): a service that does something

### Parameters:
- `param` (`int`, default: `1`): A parameter that can be set.

---
