# package
from rocket_league_drl import ROSInterface
from gym.spaces import Box

# ROS
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float
from rocket_league_msgs.msg import MatchStatus
from std_srv.srv import Empty

# System
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi

class RocketLeagueInterface(ROSInterface):
    """ROS interface for the Rocket League."""
    _node_name = "rocket_league_autonomy"
    def __init__(self):
        super().__init__()

        ## Constants
        # Actions
        self._MIN_THROTTLE_EFFORT = rospy.get_param('~effort/throttle/min', -0.25)
        self._MAX_THROTTLE_EFFORT = rospy.get_param('~effort/throttle/max',  0.25)
        self._MIN_STEERING_EFFORT = rospy.get_param('~effort/steering/min', -1.0)
        self._MAX_STEERING_EFFORT = rospy.get_param('~effort/steering/max',  1.0)

        # Observations
        self._FIELD_WIDTH = rospy.get_param('~field/width', 10)
        self._FIELD_HEIGHT = rospy.get_param('~field/height', 15)
        self._MAX_OBS_VEL = rospy.get_param('~observation/velocity/max_abs', 3.0)
        self._MAX_OBS_ANG_VEL = rospy.get_param('~observation/angular_velocity/max_abs', 2*pi)

        # Learning
        self._MAX_TIME = rospy.get_param('~max_episode_time', 30.0)
        self._CONSTANT_REWARD = rospy.get_param('~reward/constant', 0.0)
        self._BALL_DISTANCE_REWARD = rospy.get_param('~reward/ball_dist_sq', -5.0)
        self._GOAL_DISTANCE_REWARD = rospy.get_param('~reward/goal_dist_sq', -10.0)
        self._WIN_REWARD = rospy.get_param('~reward/win', 100.0)

        # Publishers
        self._throttle_pub = rospy.Publisher('effort/throttle', Float, queue_size=1)
        self._steering_pub = rospy.Publisher('effort/steering', Float, queue_size=1)
        self._reset_srv = rospy.ServiceProxy('sim_reset', Empty)

        # State variables
        self._car_odom = None
        self._ball_odom = None
        self._won = None
        self._start_time = None
        self._total_reward = 0
        self._episode = 0

        # Subscribers
        rospy.Subscriber('car0/odom', Odometry, self._car_odom_cb)
        rospy.Subscriber('ball/odom', Odometry, self._ball_odom_cb)
        rospy.Subscriber('match_status', MatchStatus, self._score_cb)

    @property
    def action_space(self):
        """The Space object corresponding to valid actions."""
        return Box(
            # throttle, steering
            low =np.array(
                self._MIN_THROTTLE_EFFORT, self._MIN_STEERING_EFFORT),
            high=np.array(
                self._MAX_THROTTLE_EFFORT, self._MAX_STEERING_EFFORT),
            dtype=np.float32)

    @property
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        return Box(
            # x, y, theta, v, omega (car)
            # x, y, vx, vy (ball)
            low=np.array(
                -self._FIELD_HEIGHT/2, -self._FIELD_WIDTH/2,
                -pi, -self._MAX_OBS_VEL, -self._MAX_OBS_ANG_VEL,
                -self._FIELD_HEIGHT/2, -self._FIELD_WIDTH/2,
                -self._MAX_OBS_VEL, -self._MAX_OBS_VEL),
            high=np.array(
                self._FIELD_HEIGHT/2, self._FIELD_WIDTH/2,
                pi, self._MAX_OBS_VEL, self._MAX_OBS_ANG_VEL,
                self._FIELD_HEIGHT/2, self._FIELD_WIDTH/2,
                self._MAX_OBS_VEL, self._MAX_OBS_VEL),
            dtype=np.float32)

    def _reset_env(self):
        """Reset environment for a new training episode."""
        self._reset_srv.call()

    def _reset_self(self):
        """Reset internally for a new episode."""
        # log data
        if self._has_state():
            self._log_data({
                "episode" : self._episode,
                "score" : self._won * 1.0,
                "duration" : (rospy.Time.now() - self._start_time).to_sec(),
                "net_reward" : self._total_reward})
            self._episode += 1

        # reset
        self._clear_state()
        self._won = None
        self._start_time = None
        self._start_time = None
        self._total_reward = 0

    def _has_state(self):
        """Determine if the new state is ready."""
        return (
            self._pose is not None and
            self._goal is not None and
            self._won is not None and
            self._alive is not None)

    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        self._pose = None
        self._goal = None
        self._won = None
        self._alive = None

    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        assert self._has_state()

        # combine car / ball odomes for observation
        car = np.asarray(self._car_odom, dtype=np.float32)
        ball = np.asarray(self._ball_odom, dtype=np.float32)
        observation = np.concatenate((car, ball))
        if not self.observation_space.contains(observation):
            rospy.logerr("observation outside of valid bounds:\nObservation: %s", observation.tostring())

        # check if time exceeded
        if self._start_time is None:
            self._start_time = rospy.Time.now()
        done = (rospy.Time.now() - self._start_time).to_sec() >= self._MAX_TIME

        # Determine reward
        reward = self._CONSTANT_REWARD

        ball_dist_sq = np.sum(np.square(ball[1:3] - car[1:3]))
        reward += self._BALL_DISTANCE_REWARD * ball_dist_sq

        goal_dist_sq = np.sum(np.square(ball[1:3] - np.array([self._FIELD_HEIGHT, 0])))
        reward += self._GOAL_DISTANCE_REWARD * goal_dist_sq

        if self._won:
            reward += self._WIN_REWARD
            done = True

        self._total_reward += reward

        return (observation, reward, done, {})

    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        assert self.action_space.contains(action)

        throttle, steering = np.split(action, action.size)

        self._throttle_pub.publish(throttle)
        self._steering_pub.publish(steering)

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
        self._won = score_msg.status == "A scored!"
        with self._cond:
            self._cond.notify_all()
