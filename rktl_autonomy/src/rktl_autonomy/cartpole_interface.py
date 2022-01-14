"""
Interface to the cart_pole_node for testing.

License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# package
from rktl_autonomy import ROSInterface
from gym.spaces import Discrete, Box

# ROS
import rospy
from std_msgs.msg import Int32, Float32, Float32MultiArray, Bool
from std_srvs.srv import Empty

# System
import numpy as np
from enum import IntEnum, unique, auto
from math import pi

@unique
class CartpoleActions(IntEnum):
    """Possible actions for deep learner."""     
    LEFT = 0
    RIGHT = auto()
    SIZE = auto()

class CartpoleInterface(ROSInterface):
    """ROS interface for the cartpole game."""
    _node_name = "cartpole_ros"
    def __init__(self):
        super().__init__()

        # Publishers
        self._action_pub = rospy.Publisher('cartpole/action', Int32, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('cartpole/reset', Empty)

        # State variables
        self._obs = None
        self._reward = None
        self._done = None

        # Subscribers
        rospy.Subscriber('cartpole/observation', Float32MultiArray, self._obs_cb)
        rospy.Subscriber('cartpole/done', Bool, self._done_cb)
        rospy.Subscriber('cartpole/reward', Float32, self._reward_cb)

    @property
    def action_space(self):
        """The Space object corresponding to valid actions."""
        return Discrete(CartpoleActions.SIZE)

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        return Box(low=-pi, high=pi, shape=(4,), dtype=np.float32)

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self._clear_state()

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
        assert self._has_state()
        return (self._obs, self._reward, self._done, {})

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < self.action_space.n
        self._action_pub.publish(action)

    def _obs_cb(self, obs_msg):
        """Callback for observation of game."""
        assert len(obs_msg.data) == self.observation_space.shape[0]
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
