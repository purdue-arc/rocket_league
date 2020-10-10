"""Contains the Renderer class.
License:
  BSD 3-Clause License
  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
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

from threading import Thread
import numpy as np
import pygame

from snakesim.geometry import Vector

class Renderer(object):
    """Render the Snake game in pygame."""
    COLOR_BACKGROUND = (200, 200, 200)  # Gray
    COLOR_GOAL = (255, 0, 0)            # Red
    COLOR_BODY = (0, 200, 0)            # Green
    COLOR_HEAD = (150, 200, 0)          # Yellow

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, bounds, padding, scaling=50):
        self.bounds = bounds
        self.padding = padding
        self.scaling = scaling
        window_size = int(scaling * (bounds + 2*padding))
        pygame.display.init()
        self._screen = pygame.display.set_mode((window_size, window_size))
        self._thread = None

    def _convert_to_display_coords(self, position):
        """Convert game coordinates to display coordinates."""
        display = position + Vector(1, 1) * self.padding
        display = np.matmul(np.array([[1, 0], [0, -1]]), display)
        display += Vector(0, self.bounds + 2*self.padding)
        display *= self.scaling
        return display.astype(np.int32)

    def render(self, goal, snake):
        """Render the current state of the game."""

        if self._thread is not None and self._thread.is_alive():
            # Still rendering the last frame, drop this one
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.COLOR_BACKGROUND)

        if goal.position is not None:
            pygame.draw.circle(
                self._screen,
                self.COLOR_GOAL,
                self._convert_to_display_coords(goal.position),
                int(goal.radius*self.scaling)
            )

        for segment in snake.body:
            pygame.draw.circle(
                self._screen,
                self.COLOR_BODY,
                self._convert_to_display_coords(segment.position),
                int(segment.radius*self.scaling)
            )

        pygame.draw.circle(
            self._screen,
            self.COLOR_HEAD,
            self._convert_to_display_coords(snake.head.position),
            int(snake.head.radius*self.scaling)
        )

        self._thread = Thread(target=pygame.display.flip)
        self._thread.start()
