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
from tf.transformations import quaternion_from_euler
import numpy

# Local classes
from tire import Tire, TireDef

class CarDef(object):
    """Holds relevent data for a car instance"""

    def __init__(self, tireDef, initPos, initAngle, max_angle, p_gain, i_gain, car_length, car_width, wheelbase,
                 bumper_width, density):

        self.initPos = initPos
        self.initAngle = initAngle
        self.tireDef = tireDef

        self.vertices = [(-0.0535, 0.075), (-0.0385, 0.09), (0.0385, 0.09), (0.0535, 0.075), (0.0535, -0.09), (-0.0535, -0.09)]
        self.tireAnchors = [(-0.0535, 0.0595), (0.0535, 0.0595), (0.0535, -0.0595), (-0.0535, -0.0595)]

        self.density = density

        self.maxAngle = math.radians(max_angle)
        self.pgain = p_gain
        self.igain = i_gain

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
            tirePos = (carDef.tireAnchors[i][0] + carDef.initPos[0],
                       carDef.tireAnchors[i][1] + carDef.initPos[1])

            tire = Tire(world, carDef.tireDef, tirePos, car_weight=self.body.mass)
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
        
        self.body.angle = carDef.initAngle
        self.maxAngle = carDef.maxAngle
        self.pgain = carDef.pgain
        self.igain = carDef.igain
        self.i_sum = 0.0

    def getPoint(self):
        return (self.body.position[0], self.body.position[1], 0)

    def getQuaternion(self):
        angle = (self.body.angle + math.pi / 2)  % (2.0 * math.pi)
        return quaternion_from_euler(0, 0, angle)

    def set_angle(self, angle):
        self._flJoint.SetLimits(angle, angle)
        self._frJoint.SetLimits(angle, angle)

    def step(self, linear_cmd, angular_cmd, dt):

        for tire in self.tires:
            tire.updateFriction()

        for tire in self.tires:
            tire.updateDrive(linear_cmd, dt)

        if linear_cmd.x != 0:
            curr_angle = self._flJoint.angle

            # The further you're off, the sharper you'll turn the tires
            turn = angular_cmd.z
            if linear_cmd.x > 0:
                turn += self.body.angularVelocity
            else:
                turn -= self.body.angularVelocity

            if turn == 0:
                self.i_sum = 0
            else:
                self.i_sum += turn
            turn *= -self.pgain
            turn -= self.i_sum * self.igain
            new_angle = curr_angle + turn

            # Turn tires at a constant rate
            # new_angle = 0
            # delta = angular_cmd.z - (-self.body.angularVelocity) * numpy.sign(linear_cmd.x)
            # if (delta > 0):
            #     new_angle = curr_angle - self.pgain * math.pi / 180
            # elif delta < 0:
            #     new_angle = curr_angle + self.pgain * math.pi / 180

            if new_angle > self.maxAngle:
                new_angle = self.maxAngle
            elif new_angle < -self.maxAngle:
                new_angle = -self.maxAngle

            self._flJoint.SetLimits(new_angle, new_angle)
            self._frJoint.SetLimits(new_angle, new_angle)