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

class World(Box2D.b2World):
    """Simulates world for car to exist in"""

    wall_thickness = 0.02

    def __init__(self, map_height, map_width):
        super(World, self).__init__()
        self.gravity = (0,0)

        self.wall_thickness = 0.02 * math.sqrt(map_width**2 + map_height**2)

        self.gndBody = self.CreateBody()
        gndShape = Box2D.b2PolygonShape(box=(map_width, map_height))
        gndFixtureDef = Box2D.b2FixtureDef(shape=gndShape)
        gndFixtureDef.isSensor = True
        self.gndBody.CreateFixture(gndFixtureDef)

        self.wallBodies = []
        wallPos = [(0, map_height), (map_width, 0), (0,0), (0,0)]

        for i in range(len(wallPos)):
            wallBody = self.CreateBody(position=wallPos[i])

            if (i % 2) == 0:
                wallShape = Box2D.b2PolygonShape(box=(map_width, self.wall_thickness))
            else:
                wallShape = Box2D.b2PolygonShape(box=(self.wall_thickness, map_height))

            wallFixtureDef = Box2D.b2FixtureDef(shape=wallShape, restitution=0.01)
            wallBody.CreateFixture(wallFixtureDef)

            self.wallBodies.append(wallBody)
