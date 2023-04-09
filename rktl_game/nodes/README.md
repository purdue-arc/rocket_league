## ROS Nodes (`nodes` directory)

These are the [ROS Nodes](http://wiki.ros.org/Nodes) provided by this package.
Most likely, you will use a launch file to run these. You can also run them
by using `rosrun`:

```bash
rosrun rktl_autonomy <Node name>
```

### The `game_manager` node

This node tracks the score of a match played by two teams (orange and blue) and updates the match
status (playing, paused, finished) based on certain conditions. It subscribes to the Odometry topic
of the ball to detect goals and publishes the match status and score to the match_status topic. The
ScoreKeeper node also provides three services (`reset_game`, `unpause_game`, `pause_game`) that
enable resetting the game, unpausing it, or pausing it, respectively.

#### Node Initialization

When this node is initialized, it sets several instance variables, including:

- `orange_score`: the score of the orange team (initialized to 0)
- `blue_score`: the score of the blue team (initialized to 0)
- `orange_goal`: the x-coordinate of the center of the orange team's goal (initialized based on the field length parameter)
- `blue_goal`: the x-coordinate of the center of the blue team's goal (initialized based on the field length parameter)
- `enabled`: a boolean flag indicating whether the game is currently enabled (initialized to False)
- `match_status`: an integer indicating the current match status (initialized to 0, which means "not started")
- `game_clock`: the time remaining in the game, in seconds (initialized based on the game length parameter)
- `manager_rate`: the rate at which the main loop runs (initialized based on the manager rate parameter)

#### ROS topics:

The `game_manager` node publishes and subscribes to the following topics:

##### Publishers:

- `match_status`: publishes the current match status and score.

##### Subscribers:

- `ball/odom`: subscribes to the position of the ball to check if a goal has been scored.
- `cars/enable`: subscribes to enable or disable the cars.

##### Services:

- `reset_game`: resets the game and sets the scores to 0.
- `unpause_game`: unpauses the game and enables the cars.
- `pause_game`: pauses the game and disables the cars.

##### ROS parameters:

- `/field/length`: the length of the playing field.
- `/game_length`: the length of the game in seconds.
- `manager_rate`: the rate at which the main loop is executed.
