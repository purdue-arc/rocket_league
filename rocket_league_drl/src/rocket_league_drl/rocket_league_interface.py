# package
from rocket_league_drl import ROSInterface

# ROS
import rospy
from nav_msgs.msg import Odometry
from rocket_league_msgs.msg import Target
from std_srv.srv import Empty

# Gym
from gym.spaces import Box

# System
import numpy as np
from tf.transformations import euler_from_quaternion
from math import exp, pi

class RocketLeagueInterface(ROSInterface):
    """ROS interface for the Rocket League."""
    def __init__(self):
        rospy.init_node('rocket_league_autonomy')

        # Constants
        self._CONST = rospy.get_param('~const', 7)

        # Publishers
        self._action_pub = rospy.Publisher('self/target', Target, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('sim/reset', Empty)

        # State variables
        self._pose = None
        self._goal = None
        self._score = None
        self._alive = None
        self._prev_time = None
        self._prev_score = None

        # Subscribers
        rospy.Subscriber('self/odom', Odometry, self._self_odom_cb)
        rospy.Subscriber('ball/odom', Odometry, self._ball_odom_cb)

    @property
    def action_size(self):
        """The Space object corresponding to valid actions."""
        return Box(
            low= np.array(0.5,  ),
            high=np.array(5.0,  ),
            dtype=np.float32)

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
        self.clear_state()
        self._prev_time = None
        self._prev_score = None

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

    def _publish_action(self, action):
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
