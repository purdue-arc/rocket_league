"""
Contains the agentInterface class.

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

from abc import ABC, abstractmethod
import numpy as np
import torch
from all.core import State

class AgentInterface(ABC):
    """
    Abstract interface for all agents to extend.

    All classes extending this for a particular agent must do the following:
    - implement all abstract methods
        - act()
        - eval()
        - save()
        - load()
    - call super().__init__() in __init__()
    """

    def __init__(self, obs_size):
        if torch.cuda.is_available():
            print("Using CUDA GPU")
            self._DEVICE = torch.device("cuda")
        else:
            print("Using CPU")
            self._DEVICE = torch.device("cpu")
        self._OBS_SIZE = obs_size

    def _convert_state(self, state):
        """Convert state from numpy to torch / ALL type."""
        obs, reward, done, __ = state

        assert np.size(obs) == self._OBS_SIZE
        obs = torch.from_numpy(obs).float().to(self._DEVICE)

        data = {
            "observation" : obs,
            "reward": reward,
            "done" : done
        }

        return State(data, self._DEVICE)

    @abstractmethod
    def act(self, state):
        """Take action during training."""
        # return super().act(self._convert_input(state))

    @abstractmethod
    def eval(self, state):
        """Take action during evaluation."""
        # return super().eval(self._convert_input(state))

    @abstractmethod
    def save(self):
        """Save weights to file."""

    @abstractmethod
    def load(self):
        """Load weights from file."""
