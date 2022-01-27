"""Interface to the Snake game in ARC Tutorials for testing.
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
from geometry_msgs.msg import Twist, PoseArray, PointStamped
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty

# System
import numpy as np
from tf.transformations import euler_from_quaternion
from enum import IntEnum, unique, auto
from math import pi

@unique
class SnakeActions(IntEnum):
    """Possible actions for deep learner."""
    FORWARD = 0
    LEFT = auto()
    RIGHT = auto()
    SIZE = auto()

class SnakeInterface(ROSInterface):
    """ROS interface for the snake game."""
    _node_name = "snake_drl"
    def __init__(self):
        super().__init__()

        # Constants
        self._NUM_SEGMENTS = rospy.get_param('~num_segments', 7)
        self._FIELD_SIZE = rospy.get_param('~field_size', 10)
        self._ANGULAR_VELOCITY = rospy.get_param('~control/max_angular_velocity', 3.0)
        self._LINEAR_VELOCITY = rospy.get_param('~control/max_linear_velocity', 3.0)
        self._DEATH_REWARD = rospy.get_param('~reward/death', 0.0)
        self._GOAL_REWARD = rospy.get_param('~reward/goal', 50.0)
        self._DISTANCE_REWARD = rospy.get_param('~reward/distance', 0.0)
        self._CONSTANT_REWARD = rospy.get_param('~reward/constant', 0.0)
        self._MAX_TIME = rospy.get_param('~max_episode_time', 30.0)

        # Publishers
        self._action_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('snake/reset', Empty)

        # State variables
        self._pose = None
        self._goal = None
        self._score = None
        self._alive = None
        self._prev_time = None
        self._start_time = None

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self._pose_cb)
        rospy.Subscriber('snake/goal', PointStamped, self._goal_cb)
        rospy.Subscriber('snake/score', Int32, self._score_cb)
        rospy.Subscriber('snake/active', Bool, self._alive_cb)

    @property
    def action_space(self):
        """The Space object corresponding to valid actions."""
        return Discrete(SnakeActions.SIZE)

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        locations = 2*(1+self._NUM_SEGMENTS)
        return Box(
            low=np.array([-pi] + locations*[0]),
            high=np.array([pi] + locations*[self._FIELD_SIZE]),
            dtype=np.float32)

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self._clear_state()
        self._prev_time = None
        self._start_time = None

    def _has_state(self):
        """Determine if the new state is ready."""
        return (
            self._pose is not None and
            self._goal is not None and
            self._score is not None and
            self._alive is not None)

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._pose = None
        self._goal = None
        self._score = None
        self._alive = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self._has_state()

        # combine pose / goal for observation
        pose = np.asarray(self._pose, dtype=np.float32)
        goal = np.asarray(self._goal, dtype=np.float32)
        observation = np.concatenate((pose, goal))

        # Determine reward and if done
        reward = 0
        done = False

        time = rospy.Time.now()
        if self._prev_time is not None:
            reward += (time - self._prev_time).to_sec() * self._CONSTANT_REWARD
            dist_sq = np.sum(np.square(pose[1:3] - goal))
            norm_dist = dist_sq / (self._FIELD_SIZE ** 2)
            reward += (time - self._prev_time).to_sec() * self._DISTANCE_REWARD * norm_dist
        self._prev_time = time

        if self._prev_score is not None:
            reward += self._GOAL_REWARD * (self._score - self._prev_score)
        self._prev_score = self._score

        if not self._alive:
            reward += self._DEATH_REWARD
            done = True

        if self._start_time is None:
            self._start_time = time

        if (time - self._start_time).to_sec() >= self._MAX_TIME:
            done = True

        # info dict
        info = {"score" : self._score}

        return (observation, reward, done, info)

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < self.action_space.n

        action_msg = Twist()
        action_msg.linear.x = self._LINEAR_VELOCITY
        if action == SnakeActions.LEFT:
            action_msg.angular.z = self._ANGULAR_VELOCITY
        if action == SnakeActions.RIGHT:
            action_msg.angular.z = -self._ANGULAR_VELOCITY

        self._action_pub.publish(action_msg)

    def _pose_cb(self, pose_msg):
        """Callback for poses of each segment of snake."""
        assert len(pose_msg.poses) == self._NUM_SEGMENTS

        yaw, __, __ = euler_from_quaternion((
            pose_msg.poses[0].orientation.x,
            pose_msg.poses[0].orientation.y,
            pose_msg.poses[0].orientation.z,
            pose_msg.poses[0].orientation.w))

        self._pose = tuple(
            [yaw] +
            [func(pose_msg.poses[i]) for i in range(self._NUM_SEGMENTS)
                for func in (
                    lambda pose: pose.position.x,
                    lambda pose: pose.position.y)])
        with self._cond:
            self._cond.notify_all()

    def _goal_cb(self, goal_msg):
        """Callback for location of goal."""
        self._goal = (goal_msg.point.x, goal_msg.point.y)
        with self._cond:
            self._cond.notify_all()

    def _score_cb(self, score_msg):
        """Callback for score of game."""
        self._score = score_msg.data
        with self._cond:
            self._cond.notify_all()

    def _alive_cb(self, alive_msg):
        """Callback for active state of snake."""
        self._alive = alive_msg.data
        with self._cond:
            self._cond.notify_all()
