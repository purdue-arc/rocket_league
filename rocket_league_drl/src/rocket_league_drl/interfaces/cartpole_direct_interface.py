#!/usr/bin/env python

# package
from rocket_league_drl.interfaces import ROSInterface

# ROS
import rospy

# System
import gym
from enum import IntEnum, unique, auto
from threading import Condition

class CartpoleDirectInterface(ROSInterface):
    """ROS interface for the cartpole game."""
    def __init__(self):
        rospy.init_node('cartpole_direct_drl')
        self._cond = Condition()
        self._RENDER = rospy.get_param('~render', False)

        self._env = gym.make('CartPole-v0')
        self._obs = self._env.reset() 
        self._reward = 0
        self._done = 0
        self._info = {}

    @property
    def OBSERVATION_SIZE(self):
        """The observation size for the network."""
        return self._env.observation_space.shape[0]

    @property
    def ACTION_SIZE(self):
        """The action size for the network."""
        return self._env.action_space.n

    def reset_env(self):
        """Reset environment for a new training episode."""
        self._obs = self._env.reset() 
        self._reward = 0
        self._done = 0
        self._info = {}

    def reset(self):
        """Reset internally for a new episode."""
        pass

    def has_state(self):
        """Determine if the new state is ready."""
        return True

    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        pass

    def get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()
        return (self._obs, self._reward, self._done, self._info)

    def publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < self.ACTION_SIZE
        if self._RENDER:
            self._env.render()
        self._obs, self._reward, self._done, self._info = self._env.step(action)
