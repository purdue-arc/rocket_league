#!/usr/bin/env python

# package
from rocket_league_drl import ROSInterface

# ROS
import rospy
from std_msgs.msg import Int32, Float32, Float32MultiArray, Bool
from std_srvs.srv import Empty

# System
import numpy as np
from enum import IntEnum, unique, auto
from threading import Condition

@unique
class CartpoleActions(IntEnum):
    """Possible actions for deep learner."""     
    LEFT = 0
    RIGHT = auto()
    SIZE = auto()

class CartpoleInterface(ROSInterface):
    """ROS interface for the cartpole game."""
    def __init__(self):
        rospy.init_node('cartpole_drl')

        # Publishers
        self._action_pub = rospy.Publisher('cartpole/action', Int32, queue_size=1)

        # Services
        self._reset_srv = rospy.ServiceProxy('cartpole/reset', Empty)

        # State variables
        self._obs = None
        self._reward = None
        self._done = None
        self._cond = Condition()

        # Subscribers
        rospy.Subscriber('cartpole/observation', Float32MultiArray, self._obs_cb)
        rospy.Subscriber('cartpole/done', Bool, self._done_cb)
        rospy.Subscriber('cartpole/reward', Float32, self._reward_cb)

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
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self.clear_state()

    def _has_state(self):
        """Determine if the new state is ready."""
        return (
            self._obs is not None and
            self._reward is not None and
            self._done is not None)

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._obs = None
        self._reward = None
        self._done = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()
        return (self._obs, self._reward, self._done, {})

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < CartpoleActions.SIZE
        self._action_pub.publish(action)

    def _obs_cb(self, obs_msg):
        """Callback for observation of game."""
        assert len(obs_msg.data) == self.OBSERVATION_SIZE
        self._obs = np.asarray(obs_msg.data, dtype=np.float32)
        with self._cond:
            self._cond.notify_all()

    def _reward_cb(self, reward_msg):
        """Callback for reward of game."""
        self._reward = reward_msg.data
        with self._cond:
            self._cond.notify_all()

    def _done_cb(self, done_msg):
        """Callback for if episode is done."""
        self._done = done_msg.data
        with self._cond:
            self._cond.notify_all()
