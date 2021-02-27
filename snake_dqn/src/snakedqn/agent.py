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
import torch.cuda.nn as nn

from collections import deque
import random

class Agent(object):
    """High level DQN controller for snake tutorial."""
    def __init__(self, state_size, action_size, training=False):
        # self.device = torch.device("cuda" if args.cuda else "cpu")

        self.state_size = state_size
        self.action_size = action_size

        self.memory = deque(maxlen=2000)
        self.gamma = 0.9
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.01
        self.learning_rate = 0.001

        self.model = nn.Sequential(
            nn.Linear(state_size, 64),
            nn.ReLu(),
            nn.Linear(64, 32),
            nn.ReLu(),
            nn.Linear(32, action_size))

        self.loss = nn.MSELoss()
        self.optimizer = nn.Adam(
            params=self.model.parameters(),
            lr=self.learning_rate)

    def remember(self, state, action, reward, next_state, done):
        """Add experience to memory dequeue for retraining later."""
        self.memory.append((state, action, reward, next_state, done))

    def _train(self, minibatch_size):
        """Update the model weights with past experiences from memory."""
        self.model.train()

        minibatch = random.sample(self.memory, minibatch_size)
        for state, action, reward, next_state, done in minibatch:
            if not done:
                reward += self.gamma * np.amax(self.model(next_state))

            optimizer.zero_grad()

            out = self.model(state)

            target = out.copy()
            target[action] = reward

            loss = self.loss(out, target)
            loss.backward()

            self.optimizer.step()

    def train(self, step, reset):
        """Train the agent using provided step and reset functions."""
        for episode in range(self.EPISODES):
            self.model.eval()
            state, __, __, __ = reset()
            done = False
            while not done:
                action = random.randint(0, self.action_size-1)
                if random.random() > self.epsilon:
                    action = self.get_action(state)
                next_state, reward, done, __ = step(action)
                self.remember(state, action, reward, next_state, done)
                
                state = next_state
                if self.epsilon > self.epsilon_min:
                    self.epsilon *= self.epsilon_decay
            self._train()

    def get_action(self, state):
        """Use forward propagation to get the predicted action from the model."""
        state = torch.cuda.from_numpy(state)
        reward = self.model(state)
        return torch.cuda.argmax(reward)
     
    def save(self, name):
        """Save weights to file."""
        torch.save(self.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        self.model.load_state_dict(torch.load(name))