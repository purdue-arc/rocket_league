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

# Third party modules
from threading import Thread
import pygame
import Box2D

class Renderer(object):
    """Render field elements of racer sim"""

    COLOR_BACKGROUND = (200, 200, 200)  # Gray
    COLOR_CAR = (255, 0, 0)             # Red
    COLOR_GOAL = (0, 200, 0)            # Green
    COLOR_BALL = (150, 200, 0)          # Yellow
    TYPE_TO_COLOR = [COLOR_CAR, COLOR_GOAL, COLOR_BALL]

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, bounds, scaling=500):
        self.bounds = bounds
        self.scaling = scaling
        self.windowSize = int(scaling * bounds)
        pygame.display.init()
        self._screen = pygame.display.set_mode((self.windowSize,
                                                self.windowSize))
        self._thread = None

    def _draw_polygon(self, body, fixture, color):
        """Draws polygons to the screen."""
        vertices = [(body.transform * v) * self.scaling \
                    for v in fixture.shape.vertices]
        vertices = [(v[0], self.windowSize - v[1]) \
                    for v in vertices]
        pygame.draw.polygon(self._screen, color, vertices)

    def _draw_circle(self, body, fixture, color):
        """Draws circles to the screen."""
        position = body.transform * fixture.shape.pos * self.scaling
        position = (position[0], self.bounds - position[1])
        pygame.draw.circle(self._screen, color, 
            [int(x) for x in position],
             int(fixture.shape.radius * self.scaling))

    def render(self, world):
        """Render the current state of the sim."""

        if self._thread is not None and self._thread.is_alive():
            # Still rendering the last frame, drop this one
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.COLOR_BACKGROUND)

        for body in world:
            for fixture in body.fixtures:
                if fixture.shape.type == Box2D.b2PolygonShape:
                    # Temporary measure to color all polygons
                    # TODO: Replace with body-based color
                    self._draw_polygon(body, fixture, self.COLOR_CAR)
                elif fixture.shape.type == Box2D.b2CircleShape:
                    self._draw_circle(body, fixture, self.COLOR_BALL)

        self._thread = Thread(target=pygame.display.flip)
        self._thread.start()