"""
Contains the DQNAgent class.

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

from all.agents import DQN
from all.approximation import QNetwork, DummyCheckpointer, PolyakTarget
from all.policies import GreedyPolicy
from all.memory import ExperienceReplayBuffer

class DQNAgent(DQN, AgentInterface):
    """DQN agent interface."""
    def __init__(self, obs_size, action_size, params):
        self._DEVICE = None
        self._init_device()

        # parameters
        LEARNING_RATE = params.get('learning_rate', 0.01)
        MEMORY_LEN = params.get('memory_len', 10000)
        GAMMA = params.get('gamma', 0.9)
        BATCH_SIZE = params.get('batch_size', 64)
        UPDATE_RATE = params.get('target_update_rate', 0.25)

        self.EPSILON_MIN = params.get('epsilon/min', 0.01)
        epsilon_max = params.get('epsilon/max', 1.0)

        epsilon_type = params.get('~epsilon/type', "exponential")
        if epsilon_type == "linear":
            self.EPSILON_DELTA = 0.0
            self.EPSILON_DECAY = params.get('~epsilon/decay', 0.99)
        elif epsilon_type == "exponential":
            self.EPSILON_DECAY = 1.0
            self.EPSILON_DELTA = params.get('~epsilon/delta', 0.005)
        else:
            print("Unknown epsilon type")
            exit()

        hidden_layers = params.get('hidden_layers', [64, 32])

        # variables
        layers = []
        for i in range(len(hidden_layers)):
            if i == 0:
                layers.append(Linear(obs_size, hidden_layers[i]))
            else:
                layers.append(Linear(hidden_layers[i-1], hidden_layers[i]))
            layers.append(ReLU())
        layers.append(Linear(hidden_layers[-1], action_size))
        model = Sequential(*layers).to(self._DEVICE)
        print(model)

        optimizer = Adam(model.parameters(), lr=LEARNING_RATE)
        net = QNetwork(model, optimizer, checkpointer=DummyCheckpointer(), target=PolyakTarget(UPDATE_RATE))
        policy = GreedyPolicy(net, action_size, epsilon_max)
        buffer = ExperienceReplayBuffer(MEMORY_LEN, self._DEVICE)
        super().__init__(
            q=net,
            policy=policy,
            replay_buffer=buffer,
            discount_factor=GAMMA,
            loss=mse_loss,
            minibatch_size=BATCH_SIZE,
            replay_start_size=BATCH_SIZE,
            update_frequency=BATCH_SIZE/2)

    def act(self, state):
        """Take action during training."""
        action = super().act(self._convert_state(state))
        if self.policy.epsilon >= (self.EPSILON_MIN + self.EPSILON_DELTA):
            self.policy.epsilon -= self.EPSILON_DELTA
        return action

    def eval(self, state):
        """Take action during evaluation."""
        return super().eval(self._convert_state(state))

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        torch.save(self.q.model.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights from {name}")
        self.q.model.model.load_state_dict(torch.load(name, map_location=self._DEVICE))
        self.q.model.model.to(self._DEVICE)
