"""
Contains the make_trainer factory.

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

from rocket_league_drl.interfaces import ROSInterface

import rospy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

import time

def makeTrainer(cls):
    """Create a trainer extending the provided agent."""
    assert issubclass(cls, ROSInterface)

    class Trainer(cls):
        """Class capable of training an agent."""
        def __init__(self):
            super().__init__()

            # Constants
            self._DELTA_T = rospy.Duration.from_sec(1.0 / rospy.get_param('~rate', 30.0))
            self._MAX_T = rospy.Duration.from_sec(rospy.get_param('~max_episode_time', 60.0))

            # Publishers
            self._clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
            self._log_pub = rospy.Publisher('training_stats', DiagnosticStatus, queue_size=1)

            self._time = rospy.Time.from_sec(time.time())

            num_episodes = rospy.get_param('~num_episodes', 10000)
            rospy.loginfo("Beginning training on " + str(num_episodes) + " episodes.")
            self.train(num_episodes)

            # idle wait to allow saving weights, etc
            rospy.loginfo("Done training.")
            rospy.spin()

        def train(self, num_episodes):
            for episode in range(num_episodes):
                # reset and get initial state
                self._full_reset()
                start_time = self._time
                obs, reward, done, __ = self.get_state()

                net_reward = reward
                steps = 0
                while not done:
                    action = self.agent.act((obs, reward, done, {}))
                    obs, reward, done, __ = self._step(action)
                    net_reward += reward
                    steps += 1

                    if self._time - start_time >= self._MAX_T:
                        done = True

                # log
                log_msg = DiagnosticStatus()
                log_msg.name = "Training"
                log_msg.values.append(KeyValue(key="episode", value=str(episode+1)))
                log_msg.values.append(KeyValue(key="net_reward", value=str(net_reward)))
                log_msg.values.append(KeyValue(key="steps", value=str(steps)))
                self._log_pub.publish(log_msg)

        def _full_reset(self):
            """Reset the agent and environment."""
            self.reset_env()
            self.reset()

            # Advance time
            self._time += self._DELTA_T
            self._clock_pub.publish(self._time)
            try:
                waits = 0
                while not self.wait_for_state():
                    self._time += self._DELTA_T
                    self._clock_pub.publish(self._time)
                    waits += 1

                    if waits > 5:
                        rospy.logerr("Failed to initialize after full reset.")
                        exit()
            except rospy.ROSInterruptException:
                pass


        def _step(self, action):
            """Step simulation one time step and return new state."""
            self.clear_state()
            self.publish_action(action)

            # Advance time
            self._time += self._DELTA_T
            self._clock_pub.publish(self._time)
            try:
                waits = 0
                while not self.wait_for_state():
                    self._time += self._DELTA_T
                    self._clock_pub.publish(self._time)
                    waits += 1

                    if waits > 1:
                        rospy.logerr("Failed to get new state.")
                        exit()
            except rospy.ROSInterruptException:
                pass

            return self.get_state()

    return Trainer
