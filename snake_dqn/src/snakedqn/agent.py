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
        if torch.cuda.is_available():
            print("Using CUDA GPU")
            self.DEVICE = torch.device("cuda")
        else:
            print("Using CPU")
            self.DEVICE = torch.device("cpu")
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
            torch.nn.Linear(self.STATE_SIZE, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, self.ACTION_SIZE)).to(self.DEVICE)
        self.loss = torch.nn.MSELoss()
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)

    def remember(self, state, action, reward, next_state, done):
        """Add experience to memory dequeue for retraining later."""
        self.memory.append((state, action, reward, next_state, done))

    def replay(self):
        """Update the model weights with past experiences from memory."""
        assert len(self.memory) >= self.BATCH_SIZE

        # get random sample
        minibatch = random.sample(self.memory, self.BATCH_SIZE)

        # extract data
        minibatch = tuple(zip(*minibatch))
        state = torch.cat(minibatch[0])
        action = torch.tensor(minibatch[1], dtype=torch.long, device=self.DEVICE)
        reward = torch.tensor(minibatch[2], dtype=torch.float, device=self.DEVICE)
        next_state = torch.cat(minibatch[3])
        done = torch.tensor(minibatch[4], dtype=torch.bool, device=self.DEVICE)

        # increase reward for incomplete states
        with torch.no_grad():
            reward = reward + self.GAMMA * ~done * torch.max(self.model(state), dim=1, keepdim=True).values.resize(self.BATCH_SIZE)

        # get output
        self.optimizer.zero_grad()
        output = self.model(state)

        # get target
        target = output.detach().clone()
        target[range(self.BATCH_SIZE), action] = reward

        # train
        loss = self.loss(output, target)
        loss.backward()
        self.optimizer.step()

        return loss.item()

    def transform_state(self, state):
        """Transform state from numpy to torch type."""
        assert np.size(state) == self.STATE_SIZE
        return torch.from_numpy(state).float().to(self.DEVICE)

    def get_action(self, state):
        """Use forward propagation to get the predicted action from the model."""
        reward = self.model(state)
        return torch.argmax(reward).item()

    def train(self, step, reset, log, episodes=1000):
        """Train the agent using provided step and reset functions."""
        print(f"Beginning training on {episodes} episodes")
        for episode in range(episodes):
            # reset episode and get initial state
            state, __, __, __ = reset()
            state = self.transform_state(state)

            done = False
            net_reward = 0
            steps = 0
            while not done:
                # compute action
                action = random.randint(0, self.ACTION_SIZE-1)
                if random.random() > self.epsilon:
                    action = self.get_action(state)

                # take action
                next_state, reward, done, __ = step(action)
                next_state = self.transform_state(next_state)

                # add to memory
                self.remember(state, action, reward, next_state, done)

                # clean up for next run
                state = next_state
                net_reward += reward
                steps += 1

            # Check for sufficient data
            if len(self.memory) >= self.BATCH_SIZE:
                # learn
                loss = self.replay()

                # log
                log({
                    "episode": episode+1,
                    "net_reward": net_reward,
                    "steps": steps,
                    "epsilon": self.epsilon,
                    "loss": loss})

            # increase exploitation vs exploration
            if self.epsilon > self.EPSILON_MIN:
                    self.epsilon *= self.EPSILON_DECAY

    def save(self, name):
        """Save weights to file."""
        print(f"Saving weights to {name}")
        torch.save(self.model.state_dict(), name)

    def load(self, name):
        """Load weights from file."""
        print(f"Loading weights from {name}")
        self.model.load_state_dict(torch.load(name, map_location=self.DEVICE))
        self.model = self.model.to(self.DEVICE)
