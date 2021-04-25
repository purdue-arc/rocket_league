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
import math

class Renderer(object):
    """Render field elements of racer sim"""

    COLOR_BACKGROUND = (200, 200, 200)  # Gray
    COLOR_CAR = (255, 0, 0)             # Red
    COLOR_TIRE = (94, 164, 191)         # Light-Blue
    COLOR_GOAL = (133, 237, 171)        # Green
    COLOR_BALL = (213, 133, 237)        # Purple
    COLOR_WALL = (112, 48, 65)          # Dark-Red
    COLOR_PNT = (245, 173, 66)          # Orange
    COLOR_LOOKAHEAD = (255, 255, 255)   # White
    COLOR_HEADLIGHTS = (0, 0, 0)        # Black

    SIZE_PNT = 0.07

    class ShutdownError(Exception):
        """Exception for when pygame is shut down"""
        pass

    def __init__(self, map_height, map_width):

        #Most people will have a 1080p monitor
        self.scaling = 1080 * 0.7 / map_height

        self.windowHeight = int(map_height * self.scaling)
        self.windowWidth = int(map_width * self.scaling)

        pygame.display.init()
        self._screen = pygame.display.set_mode((self.windowWidth, self.windowHeight))
        self._thread = None

    def _draw_polygon(self, body, fixture, color):
        """Draws polygons to the screen."""
        vertices = [(body.transform * v) * self.scaling \
                    for v in fixture.shape.vertices]
        vertices = [(v[0], v[1]) \
                    for v in vertices]

        pygame.draw.polygon(self._screen, color, vertices)

    def _draw_car(self, body, fixture, color):
        """Draws the car to the screen."""
        vertices = [(body.transform * v) * self.scaling \
                    for v in fixture.shape.vertices]
        vertices = [(v[0], v[1]) \
                    for v in vertices]

        v = vertices
        pygame.draw.polygon(self._screen, self.COLOR_HEADLIGHTS, [v[1], v[2], v[3], v[4]])
        pygame.draw.polygon(self._screen, color, [v[0], v[1], v[4], v[5]])

    def _draw_circle(self, body, fixture, color):
        """Draws circles to the screen."""
        position = body.transform * fixture.shape.pos * self.scaling

        position = (position[0], position[1])

        pygame.draw.circle(self._screen, color, 
            [int(x) for x in position],
             int(fixture.shape.radius * self.scaling))

    def _draw_pnt(self, pnt, size, color):
        """Draws point to the screen."""
        pygame.draw.circle(self._screen, color, pnt, size)

    def _visualize_point(self, color, coords, size):
        for coord in coords:
            x = int(coord[0] * self.scaling)
            y = int(coord[1] * self.scaling)

            #Gradually darkens the points
            color = (max(color[0] - 15, 0), max(color[1] - 15, 0), max(color[2] - 15, 0))

            pygame.draw.circle(self._screen, color, [x, y], size)

    def render(self, car, ball, world, lookahead, path=None, tmp=None):
        """Render the current state of the sim."""

        if self._thread is not None and self._thread.is_alive():
            # Still rendering the last frame, drop this one
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.COLOR_BACKGROUND)

        if path is not None:
            poses = []
            for posed in path:
                poses.append((posed.pose.position.x, posed.pose.position.y))
            self._visualize_point(self.COLOR_PNT, poses, int(self.SIZE_PNT * self.scaling))

        for fixture in car.body.fixtures:
            self._draw_car(car.body, fixture, self.COLOR_CAR)

        for tire in car.tires:
            for fixture in tire.body.fixtures:
                self._draw_polygon(tire.body, fixture, self.COLOR_TIRE)
        
        for fixture in ball.body.fixtures:
            self._draw_circle(ball.body, fixture, self.COLOR_BALL)
        
        for wallBody in world.wallBodies:
            for fixture in wallBody.fixtures:
                self._draw_polygon(wallBody, fixture, self.COLOR_WALL)

        for goalBody in world.goalBodies:
            for fixture in goalBody.body.fixtures:
                self._draw_polygon(goalBody.body, fixture, self.COLOR_GOAL)

        #Renders the lookahead point
        self._visualize_point(self.COLOR_LOOKAHEAD, [lookahead], int(self.SIZE_PNT * self.scaling))
        self._visualize_point(self.COLOR_BALL, [tmp], int(self.SIZE_PNT * self.scaling))

        self._thread = Thread(target=pygame.display.flip)
        self._thread.start()
