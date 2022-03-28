"""Interface to the Rocket League project.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# package
from rktl_autonomy import ROSInterface
from gym.spaces import Box, Discrete

# ROS
import rospy
from nav_msgs.msg import Odometry
from rktl_msgs.msg import ControlCommand, MatchStatus
from std_srvs.srv import Empty

# System
import numpy as np
from tf.transformations import euler_from_quaternion
from enum import IntEnum, unique, auto
from math import pi, tan

@unique
class CarActions(IntEnum):
    """Possible actions for car."""
    STOP = 0
    FWD_LEFT = auto()
    FWD_RIGHT = auto()
    FWD = auto()
    REV_LEFT = auto()
    REV_RIGHT = auto()
    REV = auto()
    SIZE = auto()

class RocketLeagueInterface(ROSInterface):
    """ROS interface for the Rocket League."""
    def __init__(self, eval=False, launch_file=['rktl_autonomy', 'rocket_league_train.launch'], launch_args=[], run_id=None):
        super().__init__(node_name='rocket_league_agent', eval=eval, launch_file=launch_file, launch_args=launch_args, run_id=run_id)

        ## Constants
        # Actions
        self._MIN_VELOCITY = -rospy.get_param('/cars/throttle/max_speed')
        self._MAX_VELOCITY =  rospy.get_param('/cars/throttle/max_speed')
        self._MIN_CURVATURE = -tan(rospy.get_param('/cars/steering/max_throw')) / rospy.get_param('cars/length')
        self._MAX_CURVATURE =  tan(rospy.get_param('/cars/steering/max_throw')) / rospy.get_param('cars/length')

        # Action space overrides
        if rospy.has_param('~action_space/velocity/min'):
            min_velocity = rospy.get_param('~action_space/velocity/min')
            assert min_velocity > self._MIN_VELOCITY
            self._MIN_VELOCITY = min_velocity
        if rospy.has_param('~action_space/velocity/max'):
            max_velocity = rospy.get_param('~action_space/velocity/max')
            assert max_velocity < self._MAX_VELOCITY
            self._MAX_VELOCITY = max_velocity
        if rospy.has_param('~action_space/curvature/min'):
            min_curvature = rospy.get_param('~action_space/curvature/min')
            assert min_curvature > self._MIN_CURVATURE
            self._MIN_CURVATURE = min_curvature
        if rospy.has_param('~action_space/curvature/max'):
            max_curvature = rospy.get_param('~action_space/curvature/max')
            assert max_curvature < self._MAX_CURVATURE
            self._MAX_CURVATURE = max_curvature

        # Observations
        self._FIELD_WIDTH = rospy.get_param('/field/width')
        self._FIELD_LENGTH = rospy.get_param('/field/length')
        self._GOAL_DEPTH = rospy.get_param('~observation/goal_depth', 0.075)
        self._MAX_OBS_VEL = rospy.get_param('~observation/velocity/max_abs', 3.0)
        self._MAX_OBS_ANG_VEL = rospy.get_param('~observation/angular_velocity/max_abs', 2*pi)

        # Learning
        self._MAX_TIME = rospy.get_param('~max_episode_time', 30.0)
        self._CONSTANT_REWARD = rospy.get_param('~reward/constant', 0.0)
        self._BALL_DISTANCE_REWARD = rospy.get_param('~reward/ball_dist_sq', 0.0)
        self._GOAL_DISTANCE_REWARD = rospy.get_param('~reward/goal_dist_sq', 0.0)
        self._WIN_REWARD = rospy.get_param('~reward/win', 100.0)
        self._LOSS_REWARD = rospy.get_param('~reward/loss', 0.0)
        self._REVERSE_REWARD = rospy.get_param('~reward/reverse', 0.0)
        self._WALL_REWARD = rospy.get_param('~reward/walls/value', 0.0)
        self._WALL_THRESHOLD = rospy.get_param('~reward/walls/threshold', 0.0)

        # Publishers
        self._command_pub = rospy.Publisher('cars/car0/command', ControlCommand, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('sim_reset', Empty)

        # State variables
        self._car_odom = None
        self._ball_odom = None
        self._score = None
        self._start_time = None

        # Subscribers
        rospy.Subscriber('cars/car0/odom', Odometry, self._car_odom_cb)
        rospy.Subscriber('ball/odom', Odometry, self._ball_odom_cb)
        rospy.Subscriber('match_status', MatchStatus, self._score_cb)

        # block until environment is ready
        rospy.wait_for_service('sim_reset')

    @property
    def action_space(self):
        """The Space object corresponding to valid actions."""
        return Discrete(CarActions.SIZE)

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        return Box(
            # x, y, theta, v, omega (car)
            # x, y, vx, vy (ball)
            low=np.array([
                -(self._FIELD_LENGTH/2) - self._GOAL_DEPTH,
                -self._FIELD_WIDTH/2, -pi,
                -self._MAX_OBS_VEL, -self._MAX_OBS_ANG_VEL,
                -(self._FIELD_LENGTH/2) - self._GOAL_DEPTH,
                -self._FIELD_WIDTH/2,
                -self._MAX_OBS_VEL, -self._MAX_OBS_VEL],
                dtype=np.float32),
            high=np.array([
                (self._FIELD_LENGTH/2) + self._GOAL_DEPTH,
                self._FIELD_WIDTH/2, pi,
                self._MAX_OBS_VEL, self._MAX_OBS_ANG_VEL,
                (self._FIELD_LENGTH/2) + self._GOAL_DEPTH,
                self._FIELD_WIDTH/2,
                self._MAX_OBS_VEL, self._MAX_OBS_VEL],
                dtype=np.float32))

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        self._clear_state()
        self._start_time = None

    def _has_state(self):
        """Determine if the new state is ready."""
        return (
            self._car_odom is not None and
            self._ball_odom is not None and
            self._score is not None)

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._car_odom = None
        self._ball_odom = None
        self._score = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self._has_state()

        # combine car / ball odoms for observation
        car = np.asarray(self._car_odom, dtype=np.float32)
        ball = np.asarray(self._ball_odom, dtype=np.float32)
        observation = np.concatenate((car, ball))
        if not self.observation_space.contains(observation):
            rospy.logerr("observation outside of valid bounds:\nObservation: %s", observation)

        # check if time exceeded
        if self._start_time is None:
            self._start_time = rospy.Time.now()
        done = (rospy.Time.now() - self._start_time).to_sec() >= self._MAX_TIME

        # Determine reward
        reward = self._CONSTANT_REWARD

        ball_dist_sq = np.sum(np.square(ball[0:2] - car[0:2]))
        reward += self._BALL_DISTANCE_REWARD * ball_dist_sq

        goal_dist_sq = np.sum(np.square(ball[0:2] - np.array([self._FIELD_LENGTH/2, 0])))
        reward += self._GOAL_DISTANCE_REWARD * goal_dist_sq

        if self._score != 0:
            done = True
            if self._score > 0:
                reward += self._WIN_REWARD
            else:
                reward += self._LOSS_REWARD

        x, y, __, v, __ = self._car_odom
        if v < 0:
            reward += self._REVERSE_REWARD
        if (abs(x) > self._FIELD_LENGTH/2 - self._WALL_THRESHOLD or
            abs(y) > self._FIELD_WIDTH/2 - self._WALL_THRESHOLD):
            reward += self._WALL_REWARD

        # info dict
        info = {'goals' : self._score}

        return (observation, reward, done, info)

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert self.action_space.contains(action)

        msg = ControlCommand()
        msg.header.stamp = rospy.Time.now()

        if (    action == CarActions.FWD or
                action == CarActions.FWD_RIGHT or
                action == CarActions.FWD_LEFT):
            msg.velocity = self._MAX_VELOCITY
        elif (  action == CarActions.REV or
                action == CarActions.REV_RIGHT or
                action == CarActions.REV_LEFT):
            msg.velocity = self._MIN_VELOCITY
        else:
            msg.velocity = 0.0

        if (    action == CarActions.FWD_LEFT or
                action == CarActions.REV_LEFT):
            msg.curvature = self._MAX_CURVATURE
        elif (  action == CarActions.FWD_RIGHT or
                action == CarActions.REV_RIGHT):
            msg.curvature = self._MIN_CURVATURE
        else:
            msg.curvature = 0.0

        self._command_pub.publish(msg)

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

        self._car_odom = (x, y, yaw, v, omega)

        with self._cond:
            self._cond.notify_all()

    def _ball_odom_cb(self, odom_msg):
        """Callback for odometry of ball."""
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y

        self._ball_odom = (x, y, vx, vy)

        with self._cond:
            self._cond.notify_all()

    def _score_cb(self, score_msg):
        """Callback for score of game."""
        if score_msg.status == MatchStatus.VICTORY_TEAM_A:
            self._score = 1
        elif score_msg.status == MatchStatus.VICTORY_TEAM_B:
            self._score = -1
        else:
            self._score = 0
        with self._cond:
            self._cond.notify_all()
