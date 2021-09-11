#!/usr/bin/env python

# package
from rocket_league_drl import ROSInterface

# ROS
import rospy

# System
import gym

class CartpoleDirectInterface(ROSInterface):
    """ROS interface for the cartpole game."""
    _node_name = "cartpole_direct"
    def __init__(self):
        super().__init__()

        self._RENDER = rospy.get_param('~render', False)

        self._env = gym.make('CartPole-v0')
        self._obs = None
        self._reward = None
        self._done = None
        self._info = None

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
        self._obs = self._env.reset()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self._obs = self._env.reset()
        self._reward = 0
        self._done = False
        self._info = {}

    def _has_state(self):
        """Determine if the new state is ready."""
        return (
            self._obs is not None and
            self._reward is not None and
            self._done is not None and
            self._info is not None)

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._obs = None
        self._reward = None
        self._done = None
        self._info = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self._has_state()
        return (self._obs, self._reward, self._done, self._info)

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        print("stepped")
        assert action >= 0 and action < self.action_space.n
        if self._RENDER:
            self._env.render()
        self._obs, self._reward, self._done, self._info = self._env.step(action)
