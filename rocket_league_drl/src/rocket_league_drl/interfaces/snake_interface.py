"""
Contains the SnakeInterface class

License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# package
from rocket_league_drl.interfaces import ROSInterface

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray, PointStamped
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty

# System
import numpy as np
from transformations import euler_from_quaternion
from enum import IntEnum, unique, auto
from math import exp
from threading import Condition

@unique
class SnakeActions(IntEnum):
    """Possible actions for deep learner."""
    FORWARD = 0
    LEFT = auto()
    RIGHT = auto()
    SIZE = auto()

class SnakeInterface(ROSInterface):
    """ROS interface for the snake game."""
    def __init__(self):
        rospy.init_node('snake_drl')

        # Constants
        self._NUM_SEGMENTS = rospy.get_param('~num_segments', 7)
        self._ANGULAR_VELOCITY = rospy.get_param('~control/max_angular_velocity', 3.0)
        self._LINEAR_VELOCITY = rospy.get_param('~control/max_linear_velocity', 3.0)
        self._DEATH_REWARD = rospy.get_param('~reward/death', 0.0)
        self._GOAL_REWARD = rospy.get_param('~reward/goal', 50.0)
        self._BASE_REWARD = rospy.get_param('~reward/distance/base', 0.0)
        self._EXP_REWARD = rospy.get_param('~reward/distance/exp', 0.0)

        # Publishers
        self._action_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('snake/reset', Empty)

        # State variables
        self._pose = None
        self._goal = None
        self._score = None
        self._alive = None
        self._prev_time = None
        self._prev_score = None
        self._cond = Condition()

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self._pose_cb)
        rospy.Subscriber('snake/goal', PointStamped, self._goal_cb)
        rospy.Subscriber('snake/score', Int32, self._score_cb)
        rospy.Subscriber('snake/active', Bool, self._alive_cb)

    @property
    def OBSERVATION_SIZE(self):
        """The observation size for the network."""
        return 3 + 2*self._NUM_SEGMENTS

    @property
    def ACTION_SIZE(self):
        """The action size for the network."""
        return SnakeActions.SIZE

    def reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def reset(self):
        """Reset internally for a new episode."""
        self.clear_state()
        self._prev_time = None
        self._prev_score = None

    def has_state(self):
        """Determine if the new state is ready."""
        return (
            self._pose is not None and
            self._goal is not None and
            self._score is not None and
            self._alive is not None)

    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._pose = None
        self._goal = None
        self._score = None
        self._alive = None

    def get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()

        # combine pose / goal for observation
        pose = np.asarray(self._pose, dtype=np.float32)
        goal = np.asarray(self._goal, dtype=np.float32)
        observation = np.concatenate((pose, goal))

        # Determine reward and if done
        reward = 0
        done = False

        time = rospy.Time.now()
        if self._prev_time is not None:
            dist = np.sqrt(np.sum(np.square(pose[1:3] - goal)))
            dist_reward_rate = self._BASE_REWARD * exp(-1.0 * self._EXP_REWARD * dist)
            reward += (time - self._prev_time).to_sec() * dist_reward_rate
        self._prev_time = time

        if self._prev_score is not None:
            reward += self._GOAL_REWARD * (self._score - self._prev_score)
        self._prev_score = self._score

        if not self._alive:
            reward += self._DEATH_REWARD
            done = True

        return (observation, reward, done, {})

    def publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < SnakeActions.SIZE

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
