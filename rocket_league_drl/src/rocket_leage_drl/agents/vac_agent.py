"""High level DQN controller for snake tutorial.

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

import numpy as np

import torch
from torch.nn import Sequential, Linear, ReLU
from torch.nn.functional import mse_loss
from torch.optim import Adam

from all.agents import VAC
from all.approximation import VNetwork, FeatureNetwork, DummyCheckpointer
from all.policies import SoftmaxPolicy
from all.core import State

class VAC_Agent(VAC):
    """High level VAC controller for snake tutorial."""
    def __init__(self, state_size, action_size,
            gamma=0.9, learning_rate=0.01,):

        # constants
        if torch.cuda.is_available():
            print("Using CUDA GPU")
            self.DEVICE = torch.device("cuda")
        else:
            print("Using CPU")
            self.DEVICE = torch.device("cpu")

        self.OBSERVATION_SIZE = state_size
        FEATURE_SIZE = 512
        FEATURE_LEARNING_RATE = learning_rate
        VALUE_LEARNING_RATE = learning_rate
        POLICY_LEARNING_RATE = learning_rate

        # feature
        feature_model = Sequential(
            Linear(self.OBSERVATION_SIZE, 512),
            ReLU(),
            Linear(512, FEATURE_SIZE),
            ReLU()).to(self.DEVICE)
        feature_optimizer = Adam(feature_model.parameters(), lr=FEATURE_LEARNING_RATE)
        feature_net = FeatureNetwork(feature_model, feature_optimizer, checkpointer=DummyCheckpointer())

        # value
        value_model = torch.nn.Sequential(
            torch.nn.Linear(FEATURE_SIZE, 512),
            torch.nn.ReLU(),
            torch.nn.Linear(512, action_size)).to(self.DEVICE)
        value_optimizer = Adam(value_model.parameters(), lr=VALUE_LEARNING_RATE)
        value_net = VNetwork(value_model, value_optimizer, checkpointer=DummyCheckpointer())

        # policy
        policy_model = Sequential(
            Linear(FEATURE_SIZE, 512),
            ReLU(),
            Linear(512, action_size)).to(self.DEVICE)
        policy_optimizer = Adam(policy_model.parameters(), lr=POLICY_LEARNING_RATE)
        # policy = GreedyPolicy(net, action_size, epsilon_max)
        policy_net = SoftmaxPolicy(policy_model, policy_optimizer, checkpointer=DummyCheckpointer())

        super().__init__(
            features=feature_net,
            v=value_net,
            policy=policy_net,
            discount_factor=gamma)

    def _convert_input(self, input):
        """Convert state from numpy to torch type."""
        observation, reward, done, __ = input

        assert np.size(observation) == self.OBSERVATION_SIZE
        observation = torch.from_numpy(observation).float().to(self.DEVICE)

        data = {
            "observation" : observation,
            "reward": reward,
            "done" : done
        }

        return State(data, self.DEVICE)

    def act(self, state):
        """Take action during training."""
        return super().act(self._convert_input(state))

    def eval(self, state):
        """Take action during evaluation."""
        return super().eval(self._convert_input(state))

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        # torch.save(self.q.model.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights from {name}")
        # self.q.model.model.load_state_dict(torch.load(name, map_location=self.DEVICE))
        # self.q.model.model.to(self.DEVICE)
