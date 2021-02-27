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
    COLOR_TIRE = (94, 164, 191)         # Light-Blue
    COLOR_GOAL = (133, 237, 171)        # Green
    COLOR_BALL = (213, 133, 237)        # Purple
    COLOR_WALL = (112, 48, 65)          # Dark-Red
    COLOR_PNT = (245, 173, 66)          # Orange
    COLOR_LOOKAHEAD = (255, 255, 255)   # White

    SIZE_PNT = 20

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
        position = (position[0], self.windowSize - position[1])
        #print position
        pygame.draw.circle(self._screen, color, 
            [int(x) for x in position],
             int(fixture.shape.radius * self.scaling))

    def _draw_pnt(self, pnt, size, color):
        """Draws point to the screen."""
        pygame.draw.circle(self._screen, color, pnt, size)

    def _visualize_point(self, color, coords, size):
        for coord in coords:
            x = int(-coord[1] * self.scaling)
            y = int(self.windowSize - coord[0] * self.scaling)
            pygame.draw.circle(self._screen, color, [x, y], size)

    def render(self, car, ball, goal, world, lookahead, path_points, path=None):
        """Render the current state of the sim."""

        if self._thread is not None and self._thread.is_alive():
            # Still rendering the last frame, drop this one
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                raise self.ShutdownError()

        self._screen.fill(self.COLOR_BACKGROUND)

        for fixture in goal.body.fixtures:
            self._draw_polygon(goal.body, fixture, self.COLOR_GOAL)

        for fixture in car.body.fixtures:
            self._draw_polygon(car.body, fixture, self.COLOR_CAR)

        for tire in car.tires:
            for fixture in tire.body.fixtures:
                self._draw_polygon(tire.body, fixture, self.COLOR_TIRE)
        
        for fixture in ball.body.fixtures:
            self._draw_circle(ball.body, fixture, self.COLOR_BALL)
        
        for wallBody in world.wallBodies:
            for fixture in wallBody.fixtures:
                self._draw_polygon(wallBody, fixture, self.COLOR_WALL)

        if path is not None:
            for posed in path:
                pose = posed.pose
                pnt = (int(pose.position.x * self.scaling), \
                       int(self.windowSize - (pose.position.y * self.scaling)))
                print(pnt)
                self._draw_pnt(pnt, self.SIZE_PNT, self.COLOR_PNT)

        #Renders the lookahead point
        self._visualize_point(self.COLOR_LOOKAHEAD, [lookahead], 10)

	#Renders the path points
	self._visualize_point(self.COLOR_PNT, path_points, 10)

        self._thread = Thread(target=pygame.display.flip)
        self._thread.start()
