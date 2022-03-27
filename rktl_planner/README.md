# Rocket League Planner

Uses classical algorithms to plan trajectories for the car and follow them.

## Constants

`path_follower.yaml`
- `lookahead_dist`: Determines lookahead circle for pure pursuit.
- `lookahead_gain`: Coefficient to grow the lookahead circle by speed.
- `lookahead_pnts`: Number of points in a path to check for intersection, -1 is all points.

`path_planner.yaml`
- `planner_type`: Determines how control points are set for a path. Can either be 'simple' or 'complex'.

## ROS Interface

You can run this package using the `simple_agent.launch` file.

In this launch file, the following nodes are launched in the `/agents/agenti` namespace:
- `path_follower`: Implements pure pursuit control to publish velocity commands from a path message.
- `bezier_path_server`: Implements a bezier curve generator. Given a start point, end point and speeds, it will generate a valid bezier curve that fits these constraints.
- `path_planner`: Implements a mechanism to select the control points for a bezier curve. Will call `bezier_path_server` to generate the path, then publishes path to a topic subscribed by `path_follower`. `path_planner` has a few different methods of selecting the path's control points, which is determined by the `planner_type` variable in the `path_planner.yaml` file.

This launch file has two arguments:
- `agent_name`: Determines namespace of this planning agent: `/agents/{agent_name}`.
- `car_name`: Car that this agent is responsible for (e.g. `/cars/{car_name}`).

In order for `path_planner` to generate a path, you need to call the `/agents/{agent_name}/reset_planner` service. This is an empty service which needs to be called each time you want a new path.

## Implementation Details

TODO

**Common Mistakes**

TODO
