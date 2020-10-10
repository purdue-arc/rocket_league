"""The classic game of snake, but in a continuous environment.
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

from Box2d import *
from pygame import draw

from racersim.tire import Tire 

class Car(object):
    """Simulates an ackerman-steering vehicle"""
    def __init__(self, world):
        self.bodyDef = b2BodyDef()
        self.bodyDef.type = b2_dynamicBody
        self.bodyDef.position = carInit
        self.body = world.CreateBody(self.bodyDef)

        vertices = [(0.0225, 0), (0.0225, 0.155),
                    (-0.0225, 0.155), (-0.0225, 0)]
        self.shape = b2PolygonShape(vertices=vertices)
        self.fixture = self.body.CreateFixture(shape=self.shape, density=0.0124)

        self.tires = []

        jointDef = b2RevoluteJointDef()
        jointDef.bodyA = self.body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        jointDef.localAnchorB.SetZero()

        tireLength = 0.037
        tireWidth = 0.0125

        tireFL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFL.body
        jointDef.localAnchorA.Set(-0.03525, 0.135)
        self.flJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFL)

        tireFR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireFR.body
        jointDef.localAnchorA.Set(0.03525, 0.135)
        self.frJoint = world.CreateJoint(jointDef)
        self.tires.append(tireFR)

        tireBL = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBL.body
        jointDef.localAnchorA.Set(-0.03525, 0.0185)
        world.CreateJoint(jointDef)
        self.tires.append(tireBL)

        tireBR = Tire(world, tireLength, tireWidth)
        jointDef.bodyB = tireBR.body
        jointDef.localAnchorA.Set(0.03525, 0.0185)
        world.CreateJoint(jointDef)
        self.tires.append(tireBR)

    def update(self, control):
        for tire in self.tires:
            tire.updateFriction()
        for tire in self.tires:
            tire.updateDrive(control)

        lockAngle = math.radians(30)
        turnSpeed = math.radians(320)
        turnPerTimeStep = turnSpeed / 60
        desiredAngle = 0

        if control[1] == 1:
            desiredAngle = lockAngle
        elif control[1] == -1:
            desiredAngle = -lockAngle

        angleNow = self.flJoint.angle
        angleToTurn = desiredAngle - angleNow
        if angleToTurn != 0:
            angleToTurn = math.copysign(turnPerTimeStep, angleToTurn)
        newAngle = angleNow + angleToTurn
        self.flJoint.SetLimits(newAngle, newAngle)
        self.frJoint.SetLimits(newAngle, newAngle)

    def draw(self, screen):
        for fixture in self.body.fixtures:
            shape = fixture.shape
            vertices = [self.body.transform * v * ppm for v in shape.vertices]
            vertices = [(v[0], v[1]) for v in vertices]
            pygame.draw.polygon(screen, (0, 25, 156), vertices)

        for tire in self.tires:
            tire.draw(screen)
