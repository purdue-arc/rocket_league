# rktl_planner

This package provides classical algorithms to plan trajectories for the car, as
well as algorithms to control the car. Both a "patrolling" planner and a planner
based on generating bezier curves are included. In this package, path
generation is handled in two parts: planner nodes decide where a car should go,
and then call upon a [server](https://wiki.ros.org/Services) node to
decide how it will get there. Currently, only 2 nodes adhere to this schema
(path_planner and bezier_path_server, respectively), but the idea is that once
multiple of these nodes are created, they should be able to be switched out
seamlessly.
