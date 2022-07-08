"""Contains the ROSInterface class, a generic implementation for binding Gym to ROS.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from abc import abstractmethod
from threading import Condition
import time, uuid, socket, os

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
    """

    def __init__(self, node_name='gym_interface', eval=False, launch_file=None, launch_args=[], run_id=None):
        """
        initializes the rospy interface
        @param node_name: desired name of this node in the ROS network
        @param eval: set true if evaluating an agent in an existing ROS env, set false if training an agent
        @param launch_file: if training, launch file to be used (ex: ['rktl_autonomy', 'rocket_league_train.launch'])
        @param launch_args: if training, arguments to be passed to roslaunch (ex: ['render:=true', rate:=10])
        @param run_id: if training, used to prevent deadlocks. if logging, run_id describes where to save files.
        """
        super().__init__()
        self.__EVAL_MODE = eval

        # ROS initialization
        if not self.__EVAL_MODE:
            assert launch_file is not None
            assert run_id is not None
            # use temporary files to enforce one environment roslaunching at a time
            while True:
                try:
                    open(f'/tmp/{run_id}_launch', mode='x')
                    break
                except FileExistsError:
                    pass
            # find a free port (using default if available)
            port = 11311
            with socket.socket() as sock:
                try:
                    # see if default port is available
                    sock.bind(('localhost', port))
                except socket.error:
                    # find a random open one
                    sock.bind(('localhost', 0))
                    port = sock.getsockname()[1]
            # launch the training ROS network
            ros_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(ros_id)
            launch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)[0]
            launch_args = [f'render:={port == 11311}', f'plot_log:={port == 11311}'] + launch_args + [
                f'agent_name:={node_name}']
            launch = roslaunch.parent.ROSLaunchParent(ros_id, [(launch_file, launch_args)], port=port)
            launch.start()
            self.close = lambda: launch.shutdown()
            # initialize self
            os.environ['ROS_MASTER_URI'] = f'http://localhost:{port}'
            rospy.init_node(node_name)
            # let someone else take a turn
            os.remove(f'/tmp/{run_id}_launch')
        else:
            # use an existing ROS network
            rospy.init_node(node_name)

        # private variables
        self._cond = Condition()
        # TODO: why cant merge to if statement above?

        # additional set up for training
        if not self.__EVAL_MODE:
            self.__DELTA_T = rospy.Duration.from_sec(1.0 / rospy.get_param('~rate', 30.0))
            self.__clock_pub = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)

            # initialize sim time
            self.__time = rospy.Time.from_sec(time.time())
            self.__clock_pub.publish(self.__time)

        # additional set up for logging
        if run_id is None:
            run_id = uuid.uuid4()
        if self.__EVAL_MODE:
            port = 11311
        self.__LOG_ID = f'{run_id}:{port}'
        self.__log_pub = rospy.Publisher('~log', DiagnosticStatus, queue_size=1)
        self.__episode = 0
        self.__net_reward = 0
        self.__start_time = rospy.Time.now()

    def step(self, action):
        """
        Implementation of gym.Env.step. This function will intentionally block
        if the ROS environment is not ready.
        @param action: action (object): an action provided by the agent
        @return: observation a tuple of the following:
            (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)

        """
        # Run one timestep of the environment's dynamics. When end of
        # episode is reached, you are responsible for calling `reset()`
        # to reset this environment's state.
        self._clear_state()
        self._publish_action(action)
        self.__step_time_and_wait_for_state()
        state = self._get_state()
        self.__net_reward += state[1]  # logging
        return state

    def reset(self):
        """
        Resets the environment to an initial state and returns an initial observation.
        Note that this function should not reset the environment's random
        number generator(s);
        random variables in the environment's state should
        be sampled independently between multiple calls to `reset()`.
        @return: the initial observation.
        """

        # Checks if have a new state ready via: _has_state
        if self._has_state():
            # gathers information: episode #, net reward, duration of the episode

            # generate log
            info = {
                'episode': self.__episode,
                'net_reward': self.__net_reward,
                'duration': (rospy.Time.now() - self.__start_time).to_sec()
            }
            # Update the message log with these parameters by publishing it

            info.update(self._get_state()[3])
            # send message
            msg = DiagnosticStatus()
            msg.level = DiagnosticStatus.OK
            msg.name = 'ROS-Gym Interface'
            msg.message = 'log of episode data'
            msg.hardware_id = self.__LOG_ID
            msg.values = [KeyValue(key=key, value=str(value)) for key, value in info.items()]
            self.__log_pub.publish(msg)
            # increment the episode count
            self.__episode += 1
            # reset the reward accumulated for the past episode
            self.__net_reward = 0

        # reset
        if not self.__EVAL_MODE:
            # reset the environment with overidden function (abstract method)
            self._reset_env()
        # reset the ROS interface (abstract method)
        self._reset_self()
        self.__step_time_and_wait_for_state(5)
        self.__start_time = rospy.Time.now()  # logging
        return self._get_state()[0]

    def __step_time_and_wait_for_state(self, max_retries=1):
        """
        Increment time and clock, take input of number of tries and do them until figure stuff out
        @param max_retries: number of time steps until state is known
        """
        if not self.__EVAL_MODE:
            self.__time += self.__DELTA_T
            self.__clock_pub.publish(self.__time)
            retries = 0
            while not self.__wait_once_for_state():

                if retries >= max_retries:
                    rospy.logerr("Failed to get new state.")
                    raise SimTimeException
                else:
                    self.__time += self.__DELTA_T
                    self.__clock_pub.publish(self.__time)
                    # increment the time if retrying wait_once_for_state
                    retries += 1
        else:
            # call for the provided number of retries
            while not self.__wait_once_for_state():
                pass  # idle wait

    def __wait_once_for_state(self):
        """ Wait and allow other threads to run."""
        with self._cond:
            has_state = self._cond.wait_for(self._has_state, 0.25)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        return has_state

    # All the below abstract methods / properties must be implemented by subclasses

    @property
    @abstractmethod
    def action_space(self):
        """ The Space object corresponding to valid actions."""

        raise NotImplementedError

    @property
    @abstractmethod
    def observation_space(self):
        """ The Space object corresponding to valid observations."""
        raise NotImplementedError

    @abstractmethod
    def _reset_env(self):
        """ Reset environment for a new episode."""
        raise NotImplementedError

    @abstractmethod
    def _reset_self(self):
        """ Reset internally for a new episode."""
        raise NotImplementedError

    @abstractmethod
    def _has_state(self):
        """ Determine if the new state is ready."""

        raise NotImplementedError

    @abstractmethod
    def _clear_state(self):
        """ Clear state variables / flags in preparation for new ones."""
        raise NotImplementedError

    @abstractmethod
    def _get_state(self):
        """ Get state tuple (observation, reward, done, info)."""
        raise NotImplementedError

    @abstractmethod
    def _publish_action(self, action):
        """ Publish an action to the ROS network."""
        raise NotImplementedError
