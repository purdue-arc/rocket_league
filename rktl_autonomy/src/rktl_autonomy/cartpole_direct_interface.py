"""
Cartpole is a very simple environment included within OpenAI Gym. There is not
much of a reason to use it, except for verifying the functionality of this
library. Two different interfaces are provided, `CartpoleInterface` and
`CartpoleDirectInterface`. The former uses the `ROSInterface` class and the
latter directly owns the Gym environment. To verify the `ROSInterface` worked,
it was confirmed that they both behave the same way.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# package
from rktl_autonomy import ROSInterface

# ROS
import rospy

# System
import gym

class CartpoleDirectInterface(ROSInterface):
    """ROS interface for the cartpole game."""
    def __init__(self, eval=False, launch_file=['rktl_autonomy', 'cartpole_train.launch'], launch_args=[], run_id=None):
        super().__init__(node_name='cartpole_agent', eval=eval, launch_file=launch_file, launch_args=launch_args, run_id=run_id)

        self._RENDER = rospy.get_param('~render', False)

        self._env = gym.make('CartPole-v0')
        self._state = None

    @property
    def action_space(self):
        """The Space object corresponding to valid actions."""
        return self._env.action_space

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        return self._env.observation_space

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._state = (self._env.reset(), 0, False, {})

    def _reset_self(self):
        """Reset internally for a new episode."""
        # Does not apply for direct interface
        pass

    def _has_state(self):
        """Determine if the new state is ready."""
        return self._state is not None

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._state = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self._has_state()
        return self._state

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < self.action_space.n
        if self._RENDER:
            self._env.render()
        self._state = self._env.step(action)
