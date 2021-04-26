#!/usr/bin/env python


# package
from rocket_league_drl.interfaces import ROSInterface    # Inherits wait_for_state

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray, PointStamped
from std_msgs.msg import Int32, Int16, Bool, Float32
#from std_srvs.srv import Empty

# System
import numpy as np
#from transformations import euler_from_quaternion
from enum import IntEnum, unique, auto
from math import exp
from threading import Condition
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


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

        # Constants
        #self._NUM_SEGMENTS = rospy.get_param('~num_segments', 7)                          Not needed for cartpole
        #self._ANGULAR_VELOCITY = rospy.get_param('~control/max_angular_velocity', 3.0)
        #self._LINEAR_VELOCITY = rospy.get_param('~control/max_linear_velocity', 3.0)
        #self._DEATH_REWARD = rospy.get_param('~reward/death', 0.0)                        May not be needed
        #self._GOAL_REWARD = rospy.get_param('~reward/goal', 50.0)
        #self._BASE_REWARD = rospy.get_param('~reward/distance/base', 0.0)
        #self._EXP_REWARD = rospy.get_param('~reward/distance/exp', 0.0)

        # Publishers
        #self._action_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)
        #self._reset_srv = rospy.ServiceProxy('snake/reset', Empty)

        self._action_pub = rospy.Publisher('cartpole/action', Int16, queue_size=1)
        self._reset_pub = rospy.Publisher('cartpole/reset', Bool, queue_size=1)                 #Does reset need to be a service?


        # State variables
        self._obs = None
        self._reward = None
        self._done = None
        self._cond = Condition()

        # Subscribers
        rospy.Subscriber('cartpole/obs', numpy_msg(Floats), self._pose_cb)   #Sending to pose_cb rather than obs_cb
        rospy.Subscriber('cartpole/done', Bool, self._done_cb)              #Sending to alive_cb rather than done_cb
        rospy.Subscriber('cartpole/reward', Float32, self._reward_cb)     #Callback may not be necessary


    @property
    def OBSERVATION_SIZE(self):
        """The observation size for the network."""
        return 4

    @property
    def ACTION_SIZE(self):
        """The action size for the network."""
        return CartpoleActions.SIZE

    def reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_pub.publish(1)

    def reset(self):
        """Reset internally for a new episode."""
        self.clear_state()

    def has_state(self):
        """Determine if the new state is ready."""
        return (
            self._obs is not None and
            self._reward is not None and
            self._done is not None)

    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._obs = None
        self._reward = None
        self._done = None

    def get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()
        return (self._obs, self._reward, self._done, {})

    def publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < CartpoleActions.SIZE
        self._action_pub.publish(action)

    def _pose_cb(self, pose_msg):
        """Callback for poses of each segment of snake."""
        assert np.size(pose_msg.data) == self.OBSERVATION_SIZE
        self._obs = pose_msg.data  #Will this work? What is the structure of numpy_msg(floats)? Seems correct based on tutorial
        with self._cond:
            self._cond.notify_all()

    def _reward_cb(self, reward_msg):
        """Callback for reward of game."""
        self._reward = reward_msg.data
        with self._cond:
            self._cond.notify_all()

    def _done_cb(self, done_msg):
        """Callback for active state of snake."""
        self._done = done_msg.data
        with self._cond:
            self._cond.notify_all()
