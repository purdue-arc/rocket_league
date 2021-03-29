"""
Contains the ROSInterface class.

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

class ROSInterface(ABC):
    """abstract interface for all wrappers to extend."""

    @abstractmethod
    def reset_env(self):
        """Reset environment for a new training episode."""
        pass
    
    @abstractmethod
    def reset(self):
        """Reset internally for a new episode."""
        pass

    @abstractmethod
    def has_state(self):
        """Determine if the new state is ready."""
        pass

    @abstractmethod
    def clear_state(self):
        """Clear state variables / flags in preparation for new ones."""
        pass

    @abstractmethod
    def get_state(self):
        """Get state tuple (observation, reward, done, info)."""
        pass

    @abstractmethod
    def publish_action(self, action):
        """Publish an action to the ROS network."""
        pass

    @abstractmethod
    @property
    def action_size(self):
        """The action size for the network."""
        pass

    @abstractmethod
    @property
    def observation_size(self):
        """The observation size for the network."""
        pass
