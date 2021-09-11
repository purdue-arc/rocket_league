"""
Contains the ROSInterface class.

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

from abc import abstractmethod
# from typing import final
from threading import Condition
import time

from gym import Env

import rospy
from rosgraph_msgs.msg import Clock


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
    _cond = Condition()

    def __init__(self):
        super().__init__()

        rospy.init_node(self._node_name)
        self.__DELTA_T = rospy.Duration.from_sec(1.0 / rospy.get_param('~rate', 30.0))
        self.__clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)

        self.__time = rospy.Time.from_sec(time.time())
        self.__clock_pub.publish(self.__time)

    # @final
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
        return self._get_state()

    # @final
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
        self._reset_env()
        self._reset_self()
        self.__step_time_and_wait_for_state(5)

    def __step_time_and_wait_for_state(self, max_retries=1):
        """Step time until a state is known."""
        self.__time += self.__DELTA_T
        self.__clock_pub.publish(self.__time)
        try:
            retries = 0
            while not self.__wait_once_for_state():
                self.__time += self.__DELTA_T
                self.__clock_pub.publish(self.__time)
                retries += 1
                if retries >= max_retries:
                    rospy.logerr("Failed to get new state.")
                    raise SimTimeException
        except rospy.ROSInterruptException:
            raise SimTimeException

    def __wait_once_for_state(self):
        """Wait and allow other threads to run."""
        with self._cond:
            has_state = self._cond.wait_for(self._has_state, 0.25)
        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()
        else:
            return has_state

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
