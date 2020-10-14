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
from tire import Tire, TireDef

class CarDef(object):
    """Holds relevent data for a car instance"""

    DEFAULT_VERTICES = [(0.0225, 0), (0.0225, 0.155),
                        (-0.0225, 0.155), (-0.0225, 0)]

    DEFAULT_ANCHORS = [(-0.03525, 0.135), (0.03525, 0.135),
                        (-0.03525, 0.0185), (0.03525, 0.0185)]

    DEFAULT_TIRE_DEF = TireDef()

    def __init__(self, initPos=(0.5, 0.5), vertices=DEFAULT_VERTICES, 
                    tireAnchors=DEFAULT_ANCHORS, tireDef=DEFAULT_TIRE_DEF,
                    density=0.0124, maxForwardSpeed=1, maxBackwardSpeed=-1,
                    maxAngle=30, turnSpeed=320):

        self.initPos = initPos
        self.vertices = vertices
        self.tireAnchors = tireAnchors
        self.tireDef = tireDef
        self.density = density
        self.maxForwardSpeed = maxForwardSpeed
        self.maxBackwardSpeed = maxBackwardSpeed
        self.maxAngle = math.radians(maxAngle)
        self.turnSpeed = math.radians(turnSpeed)

class Car(object):
    """Simulates an ackerman-steering vehicle"""

    def __init__(self, world, carDef):
        bodyDef = Box2D.b2BodyDef()
        bodyDef.type = Box2D.b2_dynamicBody
        bodyDef.position = carDef.initPos
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
            jointDef.localAnchorA.Set(carDef.tireAnchors[i][0],
                                      carDef.tireAnchors[i][1])
            
            # Store first and second joints as FL and FR respectively
            if i == 0:
                self._flJoint = world.CreateJoint(jointDef)
                self._flJoint.SetLimits(0,0)
            elif i == 1:
                self._frJoint = world.CreateJoint(jointDef)
            else:
                world.CreateJoint(jointDef)
                self._flJoint.SetLimits(0,0)
            
            self.tires.append(tire)
        
        self.maxAngle = carDef.maxAngle
        self.turnSpeed = carDef.turnSpeed

    def getPoint(self):
        return (self.body.position[0], self.body.position[1], 0)

    def getQuaternion(self):
        angle = self.body.angle % (2.0 * math.pi)
        w =  math.cos(angle/2)
        z =  math.sin(angle/2)
        mag = math.sqrt(math.pow(w,2) + math.pow(z,2))
        w /= mag
        z /= mag
        return (w, 0, 0, z)

    def step(self, linearVelocity, angularVelocity, dt):
        for tire in self.tires:
            tire.updateFriction()

        for tire in self.tires:
            tire.updateDrive(linearVelocity, dt)
        
        desiredAngleDt = angularVelocity.z * dt
        angleNow = self._flJoint.angle
        if abs(desiredAngleDt) > self.turnSpeed:
            desiredAngleDt = math.copysign(self.turnSpeed, desiredAngleDt)
        
        if abs(angleNow + desiredAngleDt) > self.maxAngle:
            angle = math.copysign(self.maxAngle, desiredAngleDt)
        else:
            angle = angleNow + desiredAngleDt

        self._flJoint.SetLimits(angle, angle)
        self._frJoint.SetLimits(angle, angle)
