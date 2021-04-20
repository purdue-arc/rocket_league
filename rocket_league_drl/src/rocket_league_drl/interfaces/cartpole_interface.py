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
class CartPoleActions(IntEnum):
    """Possible actions for deep learner."""     
    LEFT = 0
    RIGHT = auto()
    SIZE = auto()

class CartPoleInterface(ROSInterface):
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
        #self._pose = None
        #self._goal = None
        #self._score = None
        #self._alive = None
        #self._prev_time = None
        #self._prev_score = None
        #self._cond = Condition()


        self._obs = None   #In place of pose, may have to change name back to pose
        self._reward = None
        self._score = None   # Cumulative reward, need a separate self._reward?
        self._done = None  #In place of alive
        self._prev_time = None   #Unsure at the moment, _prev_ variables appear unnecessary for cartpole
        self._prev_score = None
        self._cond = Condition()         #For threading, will see how its used

        # Subscribers
        #rospy.Subscriber('snake/pose', PoseArray, self._pose_cb)
        #rospy.Subscriber('snake/goal', PointStamped, self._goal_cb)    #Not needed for cartpole
        #rospy.Subscriber('snake/score', Int32, self._score_cb)
        #rospy.Subscriber('snake/active', Bool, self._alive_cb)

        rospy.Subscriber('cartpole/obs', numpy_msg(Floats), self._pose_cb)   #Sending to pose_cb rather than obs_cb
        rospy.Subscriber('cartpole/score', Float32, self._score_cb)         #Sending to score_cb , check what they do
        rospy.Subscriber('cartpole/done', Bool, self._alive_cb)              #Sending to alive_cb rather than done_cb
        rospy.Subscriber('cartpole/reward', Float32, self._reward_cb)     #Callback may not be necessary
        #include info as state? Probably unnecessary


    @property
    def OBSERVATION_SIZE(self):
        """The observation size for the network."""
        return 4

    @property
    def ACTION_SIZE(self):
        """The action size for the network."""
        return CartPoleActions.SIZE

    def reset_env(self):       #Seems like it calls reset from gym, doesn't seem to use done message directly
        """Reset environment for a new training episode."""
        #self._reset_srv.call()   
        self._reset_pub.publish(1)  # Or is it True?

    def reset(self):                                        #Untouched, have to see what it does exactly
        """Reset internally for a new episode."""
        self.clear_state()
        self._prev_time = None
        self._prev_score = None

    def has_state(self):
        """Determine if the new state is ready."""
        return (
            self._obs is not None and
            self._reward is not None and
            self._score is not None and
            self._done is not None)

    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._obs = None
        self._reward = None
        self._score = None
        self._done = None

    def get_state(self):                 #Currently just returns info provided by subscribers, should reward be self._score or self._reward?
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()

        observation = self._obs

        # Determine reward and if done
        reward = self._reward
        done = False

        time = rospy.Time.now()
        self._prev_time = time
        self._prev_score = self._score
        done = self._done

        return (observation, reward, done, {})

    def publish_action(self, action):    #Converts action message from DRL to gym message which is just 0 (left) or 1 (right)
        """Publish an action to the ROS network."""
        assert action >= 0 and action < CartPoleActions.SIZE

        #action_msg = Twist()  Need to declare as int16?

        if action == CartPoleActions.LEFT:
            action_msg = 0
        if action == CartPoleActions.RIGHT:
            action_msg = 1

        self._action_pub.publish(action_msg)

    def _pose_cb(self, pose_msg):
        """Callback for poses of each segment of snake."""
        #assert len(pose_msg.poses) == self._NUM_SEGMENTS

        self._obs = pose_msg.data  #Will this work? What is the structure of numpy_msg(floats)? Seems correct based on tutorial
        with self._cond:
            self._cond.notify_all()

    #def _goal_cb(self, goal_msg):                               #Not needed for cartpole
    #    """Callback for location of goal."""
    #    self._goal = (goal_msg.point.x, goal_msg.point.y)
    #    with self._cond:
    #        self._cond.notify_all()

    def _score_cb(self, score_msg):
        """Callback for score of game."""
        self._score = score_msg.data
        with self._cond:                                   #Check this out
            self._cond.notify_all()

    def _reward_cb(self, reward_msg):  #Shouldn't mess with drl since it doesn't get called by it but is it needed? Use reward in _score_cb instead?
        """Callback for reward of game."""
        self._reward = reward_msg.data
        with self._cond:
            self._cond.notify_all()

    def _alive_cb(self, alive_msg):
        """Callback for active state of snake."""
        self._done = alive_msg.data
        with self._cond:
            self._cond.notify_all()
