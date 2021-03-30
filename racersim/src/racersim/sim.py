"""Contains the Sim class.

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

# 3rd party modules
import math

# Local classes
from racersim.ball import Ball
from racersim.car import Car, CarDef
from racersim.renderer import Renderer
from racersim.tire import Tire, TireDef
from racersim.world import World
import random

class Sim(object):
    """Oversees components of racersim"""

    def __init__(self, renderEnabled=True, map_height=10, map_width=100,
                 goal_width=0.125000, goal_height=0.083333,
                 velIters=6, posIters=2):

        # These positions are fully randomized
        # initPosBall = (random.uniform(0,map_width), random.uniform(0,map_height))
        # initPosCar = (random.uniform(0,map_width), random.uniform(0,map_height))

        # These positions are semi-randomized (the car doesn't need to reverse)
        # initPosBall = (random.uniform(0,map_width), 4)
        initPosCar = (random.uniform(0,map_width), 2)

        # These positions are static
        initPosBall = (2, 0.5)
        # initPosCar = (2, 2)

        self.world = World(map_height, map_width, goal_width, goal_height)

        self.renderEnabled = renderEnabled
        self.velIters = velIters
        self.posIters = posIters

        self.carDef = CarDef(initPos=initPosCar)

        self.car = Car(self.world, self.carDef)
        self.ball = Ball(self.world, initPos=initPosBall)

        self.path = None
        self.lookahead = [0, 0]

        if renderEnabled:
            self.renderer = Renderer(map_height, map_width)
            self._render()
        
        self.running = True

    def step(self, linearVelocity, angularVelocity, dt):
        """Advance one time-step in the sim."""
        if self.running:
            self.car.step(linearVelocity, angularVelocity, dt)
            self.world.Step(dt, self.velIters, self.posIters)
            self.world.ClearForces()
        
        if self.renderEnabled:
            self._render()

    def render_path(self, path):
        self.path = path

    def reset(self):
        """Reset simulator to original state."""
        self.world.DestroyBody(self.car.body)
        self.world.DestroyBody(self.ball.body)
        self.world.DestroyBody(self.goal.body)
                
        if self.carDef == None:
            if self.tireDef != None:
                self.carDef = CarDef(self.tireDef)
            else:
                self.carDef = CarDef()
        
        self.car = Car(self.world, self.carDef)
        self.ball = Ball(self.world)

        if self.renderEnabled:
            self._render()

        self.car.set_angle(0)

    def _render(self):
        """Render the current state of the sim."""
        try:
            self.renderer.render(self.car, self.ball, self.world,
                                 self.lookahead, path=self.path)
        except Renderer.ShutdownError:
            self.renderEnabled = False
