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
        super(World, self).__init__(contactListener=WorldListener(self))
        self.gravity = (0,0)

        self.gndBody = self.CreateBody()
        gndShape = Box2D.b2PolygonShape(box=(map_width, map_height))
        gndFixtureDef = Box2D.b2FixtureDef(shape=gndShape)
        gndFixtureDef.isSensor = True
        self.gndBody.CreateFixture(gndFixtureDef)

        # Setup walls
        self.wallBodies = []

        # Create top goal-surrounding walls
        width = (map_width - goal_width) / 2
        height = goal_height
        x = width/2
        y = 0
        self.createWall((x, y), width, height)
        self.createWall((map_width - x, y), width, height)

        # Create bottom goal-surrouding walls
        y = map_height
        self.createWall((x, y), width, height)
        self.createWall((map_width - x, y), width, height)

        # Create side walls
        width = goal_height
        height = map_height
        x = 0
        y = height/2
        self.createWall((x, y), width, height)
        self.createWall((map_width, y), width, height)

        # Create goals
        self.goalBodies = []
        self.goalBodies.append(Goal(self, (map_width/2, 0),
                                    goal_width, goal_height, 0))
        self.goalBodies.append(Goal(self, (map_width/2, map_height),
                                    goal_width, goal_height, 1))

        # Setup result flag
        self.winner = None
        
    def createWall(self, pos, width, height):
        wallBody = self.CreateBody(position=pos)
        wallShape = Box2D.b2PolygonShape(box=(width/2, height/2))
        wallFixtureDef = Box2D.b2FixtureDef(shape=wallShape, restitution=0.01)
        wallBody.CreateFixture(wallFixtureDef)
        self.wallBodies.append(wallBody)

class WorldListener(Box2D.b2ContactListener):
    """Handles collision and sensor events"""

    def __init__(self, world):
        Box2D.b2ContactListener.__init__(self)
        self.world = world

    def BeginContact(self, contact):
        a = contact.fixtureA
        b = contact.fixtureB

        # TODO: Add further checking for full scoring
        if a.userData != None and b.userData != None:
            aTag = a.userData.split(':')
            bTag = b.userData.split(':')
            if aTag[0] == 'ball':
                if bTag[0] == 'goal':
                    if bTag[1] == '0':
                        self.world.winner = '1'
                    else:
                        self.world.winner = '0'
            elif bTag[0] == 'ball':
                if aTag[0] == 'goal':
                    if aTag[1] == '0':
                        self.world.winner = '1'
                    else:
                        self.world.winner = '0'

    def EndContact(self, contact):
        pass