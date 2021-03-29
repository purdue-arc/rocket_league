"""
Contains the make_agent factory.

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

from rocket_league_drl import DQNAgent, RainbowDQNAgent, VACAgent, SACAgent

import rospy
from std_msgs.msg import String

def make_agent(cls):
    """Create an agent extending the provided interface."""
    class Agent(cls):
        def __init__(self):
            super().__init__(self)

            # Services
            # TODO make a service (requires custom messages)
            rospy.Subscriber('snake/dqn/load_weights', String, self._load_cb)
            rospy.Subscriber('snake/dqn/save_weights', String, self._save_cb)
       
            # DRL Agent
            agent_type = rospy.get_param('~agent_type', "dqn")
            if agent_type == "dqn":
                self.agent = DQNAgent(
                    self.observation_size,
                    self.action_size,
                    params=rospy.get_param('~'+agent_type))
            elif agent_type == "rainbow_dqn":
                self.agent = RainbowDQNAgent(
                    self.observation_size,
                    self.action_size,
                    params=rospy.get_param('~'+agent_type))
            elif agent_type == "vac":
                self.agent = VACAgent(
                    self.observation_size,
                    self.action_size,
                    params=rospy.get_param('~'+agent_type))
            elif agent_type == "sac":
                self.agent = SACAgent(
                    self.observation_size,
                    self.action_size,
                    params=rospy.get_param('~'+agent_type))
            else:
                rospy.logerr("Unknown agent type")
                exit(1)

        def _load_cb(self, load_msg):
            """Callback to load model weights."""
            self.agent.load(load_msg.data)

        def _save_cb(self, save_msg):
            """Callback to save model weights."""
            self.agent.save(save_msg.data)

    return Agent()
