# package
from rocket_league_drl import ROSInterface

# ROS
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from rocket_league_msgs.msg import Target
from std_srv.srv import Empty

# Gym
from gym.spaces import Box

# System
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi

class RocketLeagueInterface(ROSInterface):
    """ROS interface for the Rocket League."""
    def __init__(self):
        rospy.init_node('rocket_league_autonomy')

        # Constants
        self._FIELD_WIDTH = rospy.get_param('~field/width', 10)
        self._FIELD_HEIGHT = rospy.get_param('~field/height', 15)
        self._MIN_TARGET_TIME = rospy.get_param('~target/time/min', 0.5)
        self._MAX_TARGET_TIME = rospy.get_param('~target/time/max', 5.0)
        self._MIN_TARGET_VEL = rospy.get_param('~target/velocity/min', 0.1)
        self._MAX_TARGET_VEL = rospy.get_param('~target/velocity/max', 1.5)
        self._MAX_TARGET_ANG_VEL = rospy.get_param('~target/angular_velocity/max_abs', pi)
        self._MAX_OBS_VEL = rospy.get_param('~observation/velocity/max_abs', 3.0)
        self._MAX_OBS_ANG_VEL = rospy.get_param('~observation/angular_velocity/max_abs', 2*pi)
        self._MAX_TIME = rospy.get_param('~max_episode_time', 30.0)

        # Publishers
        self._target_pub = rospy.Publisher('self/target', Target, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('sim/reset', Empty)

        # State variables
        self._car_odom = None
        self._ball_odom = None
        self._score = None
        self._prev_time = None
        self._prev_score = None
        self._start_time = None

        # Subscribers
        rospy.Subscriber('self/odom', Odometry, self._car_odom_cb)
        rospy.Subscriber('ball/odom', Odometry, self._ball_odom_cb)
        rospy.Subscriber('game/score', Int32, self._score_cb)

    @property
    def action_size(self):
        """The Space object corresponding to valid actions."""
        return Box(
            # delta T, x, y, theta, v, omega
            low =np.array(self._MIN_TARGET_TIME,
                -self._FIELD_WIDTH/2, -self._FIELD_HEIGHT/2,
                -pi, self._MIN_TARGET_VEL, -self._MAX_TARGET_ANG_VEL),
            high=np.array(self._MAX_TARGET_TIME,
                self._FIELD_WIDTH/2, self._FIELD_HEIGHT/2,
                pi, self._MAX_TARGET_VEL, self._MAX_TARGET_ANG_VEL),
            dtype=np.float32)

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        return Box(
            # x, y, theta, v, omega (car)
            # x, y, vx, vy (ball)
            low=np.array(
                -self._FIELD_WIDTH/2, -self._FIELD_HEIGHT/2,
                -pi, -self._MAX_OBS_VEL, -self._MAX_OBS_ANG_VEL,
                -self._FIELD_WIDTH/2, -self._FIELD_HEIGHT/2,
                -self._MAX_OBS_VEL, -self._MAX_OBS_VEL),
            high=np.array(
                self._FIELD_WIDTH/2, self._FIELD_HEIGHT/2,
                pi, self._MAX_OBS_VEL, self._MAX_OBS_ANG_VEL,
                self._FIELD_WIDTH/2, self._FIELD_HEIGHT/2,
                self._MAX_OBS_VEL, self._MAX_OBS_VEL),
            dtype=np.float32)

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self.clear_state()
        self._prev_time = None
        self._prev_score = None
        self._start_time = None

        #@TODO log data

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

        # combine car / ball odomes for observation
        car = np.asarray(self._car_odom, dtype=np.float32)
        ball = np.asarray(self._ball_odom, dtype=np.float32)
        observation = np.concatenate((car, ball))
        assert self.observation_space.contains(observation)

        # Determine reward
        reward = 0

        time = rospy.Time.now()
        # @TODO reward
        # if self._prev_time is not None:
        #     dist = np.sqrt(np.sum(np.square(pose[1:3] - goal)))
        #     dist_reward_rate = self._BASE_REWARD * exp(-1.0 * self._EXP_REWARD * dist)
        #     reward += (time - self._prev_time).to_sec() * dist_reward_rate
        self._prev_time = time

        if self._prev_score is not None:
            reward += self._GOAL_REWARD * (self._score - self._prev_score)
        self._prev_score = self._score

        # check if time exceeded
        if self._start_time is None:
            self._start_time = time
        done = (time - self._start_time).to_sec() >= self._MAX_TIME

        return (observation, reward, done, {})

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert self.action_size.contains(action)

        delta_t, x, y, theta, v, omega = np.split(action, action.size)
        qx, qy, qz, qw = quaternion_from_euler(0, 0, theta)

        target_msg = Target()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = 'odom'         # @TODO param
        target_msg.child_frame_id = 'base_link'     # @TODO param
        target_msg.delta_t = delta_t
        target_msg.pose.position.x = x
        target_msg.pose.position.y = y
        target_msg.pose.orientation.x = qx
        target_msg.pose.orientation.y = qy
        target_msg.pose.orientation.z = qz
        target_msg.pose.orientation.w = qw
        target_msg.twist.linear.x = v
        target_msg.twist.angular.z = omega

        self._target_pub.publish(target_msg)

    def _car_odom_cb(self, odom_msg):
        """Callback for odometry of car."""
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        yaw, __, __ = euler_from_quaternion((
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w))
        v = odom_msg.twist.twist.linear.x
        omega = odom_msg.twist.twist.angular.z

        self._car_odom = (
            x, y, yaw,
            v, omega)

        with self._cond:
            self._cond.notify_all()

    def _ball_odom_cb(self, odom_msg):
        """Callback for odometry of ball."""
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y

        self._ball_odom = (
            x, y, vx, vy)

        with self._cond:
            self._cond.notify_all()

    def _score_cb(self, score_msg):
        """Callback for score of game."""
        self._score = score_msg.data
        with self._cond:
            self._cond.notify_all()
