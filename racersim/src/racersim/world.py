"""Contains the World class.

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
import Box2D
import math

# Local classes
from racersim.goal import Goal

class World(Box2D.b2World):
    """Simulates world for car to exist in"""

    def __init__(self, map_height, map_width, goal_width, goal_height):
        super(World, self).__init__(contactListener=WorldListener())
        self.gravity = (0,0)

        self.gndBody = self.CreateBody()
        gndShape = Box2D.b2PolygonShape(box=(map_width, map_height))
        gndFixtureDef = Box2D.b2FixtureDef(shape=gndShape)
        gndFixtureDef.isSensor = True
        self.gndBody.CreateFixture(gndFixtureDef)

        self.wallBodies = []

        width = (map_width - goal_width) / 4
        height = goal_height

        self.createWall((width, 0), width, height)
        self.createWall((map_width - width, 0), width, height)
        self.createWall((width, map_height), width, height)
        self.createWall((map_width - width, map_height), width, height)

        width = goal_height
        height = map_height / 2

        self.createWall((0, height), width, height)
        self.createWall((map_width, height), width, height)

        self.goalBodies = []
        self.goalBodies.append(Goal(self, (map_width/2, 0),
                                    goal_width, goal_height, 0))
        self.goalBodies.append(Goal(self, (map_width/2, map_height),
                                    goal_width, goal_height, 1))
        
    def createWall(self, pos, width, height):
        wallBody = self.CreateBody(position=pos)
        wallShape = Box2D.b2PolygonShape(box=(width, height))
        wallFixtureDef = Box2D.b2FixtureDef(shape=wallShape, restitution=0.01)
        wallBody.CreateFixture(wallFixtureDef)
        self.wallBodies.append(wallBody)

class WorldListener(Box2D.b2ContactListener):
    """Handles collision and sensor events"""

    def __init__(self):
        Box2D.b2ContactListener.__init__(self)

    def BeginContact(self, contact):
        a = contact.fixtureA
        b = contact.fixtureB
        if a.userData != None:
            print(a.userData)
        if b.userData != None:
            print(b.userData)
        pass

    def EndContact(self, contact):
        pass