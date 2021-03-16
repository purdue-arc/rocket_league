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
import all

import random

class Agent(all.agents.DQN):
    """High level DQN controller for snake tutorial."""
    def __init__(self, state_size, action_size,
            memory_len=2000, gamma=0.9, epsilon_max=1.0,
            epsilon_min_episode=1000, epsilon_min=0.01,
            learning_rate=0.01, batch_size=64, epochs=1):

        # constants
        if torch.cuda.is_available():
            print("Using CUDA GPU")
            self.DEVICE = torch.device("cuda")
        else:
            print("Using CPU")
            self.DEVICE = torch.device("cpu")
        self.STATE_SIZE = state_size
        self.EPSILON_DELTA = (epsilon_max - epsilon_min) / float(epsilon_min_episode)
        self.EPSILON_MIN = epsilon_min

        # variables
        model = torch.nn.Sequential(
            torch.nn.Linear(self.STATE_SIZE, 512),
            torch.nn.ReLU(),
            torch.nn.Linear(512, 512),
            torch.nn.ReLU(),
            torch.nn.Linear(512, 512),
            torch.nn.ReLU(),
            torch.nn.Linear(512, action_size)).to(self.DEVICE)
        optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)
        net = all.approximation.QNetwork(model, optimizer)
        policy = all.policies.GreedyPolicy(net, action_size, self.epsilon)
        buffer = all.memory.ExperienceReplayBuffer(memory_len, self.DEVICE)
        super.__init__(
            q=net,
            policy=policy,
            replay_buffer=buffer,
            discount_factor=gamma,
            loss=torch.nn.functional.mse_loss,
            minibatch_size=batch_size,
            replay_start_size=batch_size,
            update_frequency=batch_size/2)

    def _convert_input(self, state):
        """Convert state from numpy to torch type."""
        assert np.size(state) == self.STATE_SIZE
        return torch.from_numpy(state).float().to(self.DEVICE)

    def act(self, state):
        """Take action during training."""
        action = super.act(self._convert_input(state))
        if self.policy.epsilon >= (self.EPSILON_MIN + self.EPSILON_DELTA):
            self.policy.epsilon -= self.EPSILON_DELTA
        return action

    def eval(self, state):
        """Take action during evaluation."""
        return super.eval(self._convert_input(state))

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        torch.save(self.q.model.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights from {name}")
        self.q.model.model.load_state_dict(torch.load(name, map_location=self.DEVICE))
        self.q.model.model.to(self.DEVICE)
