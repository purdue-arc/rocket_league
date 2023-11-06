# rktl_control

This package performs several jobs that allow it to move physical cars according
to commands from either the planning or autonomy code.

Specifically it has three main components:

- It filters the raw perception data using either a moving average filter or a
    particle filter, while also producing estimated odometry (which includes
    velocity in addition to the position given by the raw perception data)
- It has a closed loop controller which reduces the error between the desired
    and estimated motion of the car by producing control signals.
- It has a hardware interface, which sends those control signals to the physical
    cars.

It also contains several MATLAB scripts that are useful in tuning and validating
the Python code used for this project.
