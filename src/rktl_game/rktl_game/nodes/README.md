# ROS Nodes

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun rktl_autonomy <Node name>
```

```{contents} ROS Nodes in the package
:depth: 2
:backlinks: top
:local: true
```

---

## game_manager

This node tracks the score of a match played by two teams (orange and blue) and
updates the match status (playing, paused, finished) based on certain conditions.
It subscribes to the Odometry topic of the ball to detect goals and publishes
the match status and score to the match_status topic. The ScoreKeeper node also
provides three services (`reset_game`, `unpause_game`, `pause_game`) that enable
resetting the game, unpausing it, or pausing it, respectively.

### Subscribed Topics:

- `ball/odom` ([nav_msgs/Odometry.msg](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)):
    Odometry of the ball.
- `cars/enable`([std_msgs/Bool.msg](https://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html)):
    Status of the cars (enabled/disabled).

### Published Topics:

- `match_status` ([rktl_msgs/MatchStatus](/rktl_msgs/html/msg/MatchStatus.html#http://)):
    Publishes the current match status and score.

### Services :

- `reset_game` ([std_srvs/Empty.srv](https://docs.ros.org/en/api/std_srvs/html/srv/Empty.html)):
    Resets the game and sets the scores to 0.
- `unpause_game` ([std_srvs/Empty.srv](https://docs.ros.org/en/api/std_srvs/html/srv/Empty.html)):
    Unpauses the game and enables the cars.
- `pause_game` ([std_srvs/Empty.srv](https://docs.ros.org/en/api/std_srvs/html/srv/Empty.html)):
    Pauses the game and disables the cars.

### Parameters:

- `/field/length` (`double`, default: `1.0`): The length of the playing field.
- `/game_length` (`int`, default: `90`): The length of the game in seconds.
- `manager_rate` (`int`, default: `10`): The rate at which the main loop is
    executed.

---
