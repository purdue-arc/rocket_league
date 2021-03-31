"""
Contains the VACAgent class.

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

from rocket_league_drl.agents import AgentInterface

import torch
from torch.nn import Sequential, Linear, ReLU
from torch.nn.functional import mse_loss
from torch.optim import Adam

from all.agents import VAC
from all.approximation import VNetwork, FeatureNetwork, DummyCheckpointer
from all.policies import SoftmaxPolicy

class VACAgent(VAC, AgentInterface):
    """High level VAC controller for snake tutorial."""
    def __init__(self, obs_size, action_size, params):
        self._DEVICE = None
        self._init_device()

        # parameters
        GAMMA = params.get('gamma', 0.9)
        FEATURE_LEARNING_RATE = params.get('learning_rate', 0.01)
        VALUE_LEARNING_RATE = params.get('learning_rate', 0.01)
        POLICY_LEARNING_RATE = params.get('learning_rate', 0.01)

        FEATURE_SIZE = 512

        # feature
        feature_model = Sequential(
            Linear(obs_size, 512),
            ReLU(),
            Linear(512, FEATURE_SIZE),
            ReLU()).to(self._DEVICE)
        feature_optimizer = Adam(feature_model.parameters(), lr=FEATURE_LEARNING_RATE)
        feature_net = FeatureNetwork(feature_model, feature_optimizer, checkpointer=DummyCheckpointer())

        # value
        value_model = torch.nn.Sequential(
            torch.nn.Linear(FEATURE_SIZE, 512),
            torch.nn.ReLU(),
            torch.nn.Linear(512, action_size)).to(self._DEVICE)
        value_optimizer = Adam(value_model.parameters(), lr=VALUE_LEARNING_RATE)
        value_net = VNetwork(value_model, value_optimizer, checkpointer=DummyCheckpointer())

        # policy
        policy_model = Sequential(
            Linear(FEATURE_SIZE, 512),
            ReLU(),
            Linear(512, action_size)).to(self._DEVICE)
        policy_optimizer = Adam(policy_model.parameters(), lr=POLICY_LEARNING_RATE)
        policy_net = SoftmaxPolicy(policy_model, policy_optimizer, checkpointer=DummyCheckpointer())

        super().__init__(
            features=feature_net,
            v=value_net,
            policy=policy_net,
            discount_factor=GAMMA)

    def act(self, state):
        """Take action during training."""
        return super().eval(self._convert_state(state))

    def eval(self, state):
        """Take action during evaluation."""
        return super().eval(self._convert_state(state))

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        # torch.save(self.q.model.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights from {name}")
        # self.q.model.model.load_state_dict(torch.load(name, map_location=self._DEVICE))
        # self.q.model.model.to(self._DEVICE)
