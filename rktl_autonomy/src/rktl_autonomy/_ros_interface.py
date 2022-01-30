"""Contains the ROSInterface class, a generic implementation for binding Gym to ROS.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from abc import abstractmethod
from threading import Condition, Lock
import time, uuid, socket

from gym import Env

import rospy, roslaunch
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class SimTimeException(Exception):
    """For when advancing sim time does not go as planned."""
    pass

class ROSInterface(Env):
    """Extension of the Gym environment class for all specific interfaces
    to extend. This class handles logic regarding timesteps in ROS, and
    allows users to treat any ROS system as a Gym environment once the
    interface is created.

    All classes extending this for a particular environment must do the following:
        - implement all abstract properties:
            - action_space
            - observation_space
        - implement all abstract methods:
            - _reset_env()
            - _reset_self()
            - _has_state()
            - _clear_state()
            - _get_state()
            - _publish_action()
        - notify _cond when _has_state() may have turned true
        - optionally override _node_name
    """

    _node_name = "gym_interface"

    # static vars to allow simultaneous environments
    __init_lock = Lock()
    __env_count = 0

    def __init__(self):
        super().__init__()

        # rospy.init_node(self._node_name)
        # self.__EVAL_MODE = rospy.get_param('~eval_mode', False)
        self.__EVAL_MODE = False
        self._cond = Condition()

        if not self.__EVAL_MODE:
            # launch ROS network
            with ROSInterface.__init_lock:
                ROSInterface.__env_count += 1
                port = 11311
                if self.__env_count > 1:
                    # find a free port for the ROS master
                    with socket.socket() as sock:
                        sock.bind(('localhost', 0))
                        port = sock.getsockname()[1]
                # roslaunch
                ros_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                roslaunch.configure_logging(ros_uuid)
                cli_args = ['rktl_autonomy', 'rocket_league_train.launch', f'--port {port}']
                file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
                args = cli_args[2:]
                launch = roslaunch.parent.ROSLaunchParent(ros_uuid, [(file, args)])
                launch.start()

            self.__DELTA_T = rospy.Duration.from_sec(1.0 / rospy.get_param('~rate', 30.0))
            self.__clock_pub = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)

            # initialize sim time
            self.__time = rospy.Time.from_sec(time.time())
            self.__clock_pub.publish(self.__time)

        # logging
        self.__UUID = str(uuid.uuid4())
        self.__log_pub = rospy.Publisher('~log', DiagnosticStatus, queue_size=1)
        self.__episode = 0
        self.__net_reward = 0
        self.__start_time = rospy.Time.now()

    def step(self, action):
        """
        Implementation of gym.Env.step. This function will intentionally block
        if the ROS environment is not ready.

        Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.
        Accepts an action and returns a tuple (observation, reward, done, info).
        Args:
            action (object): an action provided by the agent
        Returns:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        self._clear_state()
        self._publish_action(action)
        self.__step_time_and_wait_for_state()
        state = self._get_state()
        self.__net_reward += state[1]   # logging
        return state

    def reset(self):
        """Resets the environment to an initial state and returns an initial observation.

        Note that this function should not reset the environment's random
        number generator(s); random variables in the environment's state should
        be sampled independently between multiple calls to `reset()`. In other
        words, each call of `reset()` should yield an environment suitable for
        a new episode, independent of previous episodes.
        Returns:
            observation (object): the initial observation.
        """
        # logging
        if self._has_state():
            # generate log
            info = {
                "episode"    : self.__episode,
                "net_reward" : self.__net_reward,
                "duration"   : (rospy.Time.now() - self.__start_time).to_sec()
            }
            info.update(self._get_state()[3])
            # send message
            msg = DiagnosticStatus()
            msg.level = DiagnosticStatus.OK
            msg.name = self._node_name
            msg.message = "logged data"
            msg.hardware_id = self.__UUID
            msg.values = [KeyValue(key=key, value=str(value)) for key, value in info.items()]
            self.__log_pub.publish(msg)
            # update variables (update time after reset)
            self.__episode += 1
            self.__net_reward = 0

        # reset
        if not self.__EVAL_MODE:
            self._reset_env()
        self._reset_self()
        self.__step_time_and_wait_for_state(5)
        self.__start_time = rospy.Time.now()    # logging
        return self._get_state()[0]

    def __step_time_and_wait_for_state(self, max_retries=1):
        """Step time until a state is known."""
        if not self.__EVAL_MODE:
            self.__time += self.__DELTA_T
            self.__clock_pub.publish(self.__time)
            retries = 0
            while not self.__wait_once_for_state():
                self.__time += self.__DELTA_T
                self.__clock_pub.publish(self.__time)
                retries += 1
                if retries >= max_retries:
                    rospy.logerr("Failed to get new state.")
                    raise SimTimeException
        else:
            while not self.__wait_once_for_state():
                pass    # idle wait

    def __wait_once_for_state(self):
        """Wait and allow other threads to run."""
        with self._cond:
            has_state = self._cond.wait_for(self._has_state, 0.25)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        return has_state

    def get_run_uuid(self):
        """Get the uuid associated with this run."""
        return self.__UUID

    # All the below abstract methods / properties must be implemented by subclasses
    @property
    @abstractmethod
    def action_space(self):
        """The Space object corresponding to valid actions."""
        raise NotImplementedError

    @property
    @abstractmethod
    def observation_space(self):
        """The Space object corresponding to valid observations."""
        raise NotImplementedError

    @abstractmethod
    def _reset_env(self):
        """Reset environment for a new episode."""
        raise NotImplementedError

    @abstractmethod
    def _reset_self(self):
        """Reset internally for a new episode."""
        raise NotImplementedError

    @abstractmethod
    def _has_state(self):
        """Determine if the new state is ready."""
        raise NotImplementedError

    @abstractmethod
    def _clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        raise NotImplementedError

    @abstractmethod
    def _get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        raise NotImplementedError

    @abstractmethod
    def _publish_action(self, action):
        """Publish an action to the ROS network."""
        raise NotImplementedError
