"""
Contains the CartPoleInterface class

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
from rocket_leage_drl import ROSInterface

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseArray, PointStamped
from std_msgs.msg import Int32, Bool
from std_srvs.srv import Empty

# System
import numpy as np
from transformations import euler_from_quaternion
from enum import IntEnum, unique, auto
from threading import Lock, Condition
from math import exp

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
        self.NUM_SEGMENTS = rospy.get_param('~num_segments', 7)
        self.ANGULAR_VELOCITY = rospy.get_param('~control/max_angular_velocity', 3.0)
        self.LINEAR_VELOCITY = rospy.get_param('~control/max_linear_velocity', 3.0)
        self.DEATH_REWARD = rospy.get_param('~reward/death', 0.0)
        self.GOAL_REWARD = rospy.get_param('~reward/goal', 50.0)
        self.BASE_REWARD = rospy.get_param('~reward/distance/base', 0.0)
        self.EXP_REWARD = rospy.get_param('~reward/distance/exp', 0.0)

        # Publishers
        self.action_pub = rospy.Publisher('snake/cmd_vel', Twist, queue_size=1)
        self.reset_srv = rospy.ServiceProxy('snake/reset', Empty)

        # State variables
        self.pose = None
        self.goal = None
        self.score = None
        self.alive = None
        self.prev_score = None
        self.cond = Condition()

        # Subscribers
        rospy.Subscriber('snake/pose', PoseArray, self._pose_cb)
        rospy.Subscriber('snake/goal', PointStamped, self._goal_cb)
        rospy.Subscriber('snake/score', Int32, self._score_cb)
        rospy.Subscriber('snake/active', Bool, self._alive_cb)

    def reset_env(self):
        """Reset environment for a new training episode."""
        self.reset_srv.call()

    def reset(self):
        """Reset internally for a new episode."""
        self.clear_state()
        self.prev_score = None

    def has_state(self):
        """Determine if the new state is ready."""
        return (
            self.pose is not None and
            self.goal is not None and
            self.score is not None and
            self.alive is not None)

    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self.pose = None
        self.goal = None
        self.score = None
        self.alive = None

    def get_state(self, time):
        """Get state tuple (observation, reward, done, info)."""
        assert self.has_state()

        # combine pose / goal for observation
        pose = np.asarray(self.pose, dtype=np.float32)
        goal = np.asarray(self.goal, dtype=np.float32)
        observation = np.concatenate((pose, goal))

        # Determine reward and if done
        reward = 0
        done = False

        dist = np.sqrt(np.sum(np.square(pose[1:3] - goal)))
        dist_reward = self.BASE_REWARD * exp(-1.0 * self.EXP_REWARD * dist)
        reward += self.DIST_REWARD_SCALE * dist_reward

        if self.prev_score is not None:
            reward += self.GOAL_REWARD * (self.score - self.prev_score)
        self.prev_score = self.score

        if not self.alive:
            reward += self.DEATH_REWARD
            done = True

        return (observation, reward, done, {})

    def publish_action(self, action):
        """Publish an action to the ROS network."""
        assert action >= 0 and action < SnakeActions.SIZE

        action_msg = Twist()
        action_msg.linear.x = self.LINEAR_VELOCITY
        if action == SnakeActions.LEFT:
            action_msg.angular.z = self.ANGULAR_VELOCITY
        if action == SnakeActions.RIGHT:
            action_msg.angular.z = -self.ANGULAR_VELOCITY

        self.action_pub.publish(action_msg)

    def _pose_cb(self, pose_msg):
        """Callback for poses of each segment of snake."""
        assert len(pose_msg.poses) == self.NUM_SEGMENTS

        yaw, __, __ = euler_from_quaternion((
            pose_msg.poses[0].orientation.x,
            pose_msg.poses[0].orientation.y,
            pose_msg.poses[0].orientation.z,
            pose_msg.poses[0].orientation.w))

        self.pose = tuple(
            [yaw] +
            [func(pose_msg.poses[i]) for i in range(self.NUM_SEGMENTS)
                for func in (
                    lambda pose: pose.position.x,
                    lambda pose: pose.position.y)])
        with self.cond:
            self.cond.notify_all()

    def _goal_cb(self, goal_msg):
        """Callback for location of goal."""
        self.goal = (goal_msg.point.x, goal_msg.point.y)
        with self.cond:
            self.cond.notify_all()

    def _score_cb(self, score_msg):
        """Callback for score of game."""
        self.score = score_msg.data
        with self.cond:
            self.cond.notify_all()

    def _alive_cb(self, alive_msg):
        """Callback for active state of snake."""
        self.alive = alive_msg.data
        with self.cond:
            self.cond.notify_all()
