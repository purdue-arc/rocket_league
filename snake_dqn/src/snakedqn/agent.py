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

from collections import deque
import random

class Agent(object):
    """High level DQN controller for snake tutorial."""
    def __init__(self, state_size, action_size,
            memory_len=2000, gamma=0.9,
            epsilon_decay=0.99, epsilon_min=0.01,
            learning_rate=0.01, batch_size=64):

        # constants
        self.DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.STATE_SIZE = state_size
        self.ACTION_SIZE = action_size
        self.GAMMA = gamma
        self.EPSILON_DECAY = epsilon_decay
        self.EPSILON_MIN = epsilon_min
        self.BATCH_SIZE = batch_size

        # variables
        self.epsilon = 1.0
        self.memory = deque(maxlen=memory_len)
        self.model = torch.nn.Sequential(
            torch.nn.Linear(self.STATE_SIZE, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, self.ACTION_SIZE)).to(self.DEVICE)
        self.loss = torch.nn.MSELoss()
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)

    def remember(self, state, action, reward, next_state, done):
        """Add experience to memory dequeue for retraining later."""
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        """Update the model weights with past experiences from memory."""
        self.model.train()

        # Check for sufficient data
        if self.BATCH_SIZE > len(self.memory):
            return

        minibatch = random.sample(self.memory, self.BATCH_SIZE)
        for state, action, reward, next_state, done in minibatch:
            if not done:
                reward += self.GAMMA * torch.max(self.model(next_state)).item()

            self.optimizer.zero_grad()

            out = self.model(state)

            target = out.detach().clone()
            target[action] = reward

            loss = self.loss(out, target)
            loss.backward()

            self.optimizer.step()

    def transform_state(self, state):
        """Transform state from numpy to torch type."""
        assert np.size(state) == self.STATE_SIZE
        return torch.from_numpy(state).to(self.DEVICE).float()

    def get_action(self, state):
        """Use forward propagation to get the predicted action from the model."""
        self.model.eval()
        reward = self.model(state)
        return torch.argmax(reward).item()

    def train(self, step, reset, log, episodes=1000):
        """Train the agent using provided step and reset functions."""
        for episode in range(episodes):
            # reset episode and get initial state
            state, __, __, __ = reset()
            state = self.transform_state(state)

            done = False
            net_reward = 0
            steps = 0
            while not done:
                # compute action
                action = random.randint(0, self.action_size-1)
                if random.random() > self.epsilon:
                    action = self.get_action(state)

                # take action
                next_state, reward, done, __ = step(action)
                next_state = self.transform_state(next_state)

                # add to memory and learn
                self.remember(state, action, reward, next_state, done)
                self.replay()

                # clean up for next run
                state = next_state
                net_reward += reward
                steps += 1
                if self.epsilon > self.EPSILON_MIN:
                    self.epsilon *= self.EPSILON_DECAY

            # log
            log(episode, net_reward, steps, self.epsilon)

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        torch.save(self.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights fron {name}")
        self.model.load_state_dict(torch.load(name))
