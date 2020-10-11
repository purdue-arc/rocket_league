"""Contains the Car and CarDef class.

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
from tire import Tire 

class CarDef(object):
    """Holds relevent data for a car instance"""

    def __init__(self, vertices, tireAnchors, tireDef, density=0.0124,
                    maxForwardSpeed=1, maxBackwardSpeed=-1):
        
        self.vertices = vertices
        self.tireAnchors = tireAnchors
        self.tireDef = tireDef
        self.density = density
        self.maxForwardSpeed = maxForwardSpeed
        self.maxBackwardSpeed = maxBackwardSpeed

class Car(object):
    """Simulates an ackerman-steering vehicle"""

    def __init__(self, world, x, y, carDef, timestep):
        bodyDef = Box2D.b2BodyDef()
        bodyDef.type = Box2D.b2_dynamicBody
        bodyDef.position = (x, y)
        self.body = world.CreateBody(bodyDef)

        shape = Box2D.b2PolygonShape(vertices=carDef.vertices)
        self.body.CreateFixture(shape=shape, density=carDef.density)

        jointDef = Box2D.b2RevoluteJointDef()
        jointDef.bodyA = self.body
        jointDef.enableLimit = True
        jointDef.lowerAngle = 0
        jointDef.upperAngle = 0
        jointDef.localAnchorB.SetZero()

        self.tires = []
        for i in range(len(carDef.tireAnchors)):

            tire = Tire(world, carDef.tireDef, 
                        maxForwardSpeed=carDef.maxForwardSpeed,
                        maxBackwardSpeed=carDef.maxBackwardSpeed)
            jointDef.bodyB = tire.body
            jointDef.localAnchorA.Set(carDef.tireAnchors[i])
            
            # Store first and second joints as FL and FR respectively
            if i == 0:
                self._flJoint = world.CreateJoint(jointDef)
            elif i == 1:
                self._frJoint = world.CreateJoint(jointDef)
            else:
                world.CreateJoint(jointDef)
            
            self.tires.append(tire)
        
        self.maxAngle = carDef.maxAngle
        self.turnPerTimeStep = carDef.turnSpeed / timestep

    def update(self, command):
        for tire in self.tires:
            tire.updateFriction()
        for tire in self.tires:
            tire.updateDrive(command.linear.Y)

        desiredAngle = command.angular.Z
        if abs(desiredAngle) > self.maxAngle:
            desiredAngle = math.copysign(self.maxAngle, desiredAngle)

        angleNow = self._flJoint.angle
        angleToTurn = desiredAngle - angleNow
        if angleToTurn != 0 and abs(angleToTurn) > self.turnPerTimeStep:
            angleToTurn = math.copysign(self.turnPerTimeStep, angleToTurn)
        newAngle = angleNow + angleToTurn

        self._flJoint.SetLimits(newAngle, newAngle)
        self._frJoint.SetLimits(newAngle, newAngle)