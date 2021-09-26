"""Contains the Tire and TireDef class.

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

class TireDef(object):
    """Holds relevent data for a tire instance"""
    def __init__(self, width, length, max_lateral_impulse, max_drive_force,
                 drag_force_coeff, angular_impulse_coeff, density):

        self.width = width
        self.length = length
        self.density = density

        self.maxLateralImpulse = max_lateral_impulse
        self.maxDriveForce = max_drive_force
        self.dragForceCoeff = drag_force_coeff
        self.angularImpulseCoeff = angular_impulse_coeff

class Tire(object):
    """Simulates a single tire of a vehicle"""
    def __init__(self, world, tireDef, position, car_weight=0.200):
        bodyDef = Box2D.b2BodyDef()
        bodyDef.type = Box2D.b2_dynamicBody
        bodyDef.position = position
        self.body = world.CreateBody(bodyDef)

        shape = Box2D.b2PolygonShape(box=(tireDef.width/2, tireDef.length/2))
        self.body.CreateFixture(shape=shape, density=tireDef.density)

        self.car_weight = car_weight
        self.maxDriveForce = tireDef.maxDriveForce
        self.maxLateralImpulse = tireDef.maxLateralImpulse
        self.dragForceCoeff = tireDef.dragForceCoeff
        self.angularImpulseCoeff = tireDef.angularImpulseCoeff
        self.density = tireDef.density

    def getForwardVelocity(self):
        normal = self.body.GetWorldVector((0,1))
        return Box2D.b2Dot(normal, self.body.linearVelocity) * normal

    def getLateralVelocity(self):
        normal = self.body.GetWorldVector((1,0))
        return Box2D.b2Dot(normal, self.body.linearVelocity) * normal

    def updateFriction(self):
        impulse = self.body.mass * -self.getLateralVelocity()
        if impulse.length > self.maxLateralImpulse:
            impulse *= self.maxLateralImpulse / impulse.length
        self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, wake=True)
        self.body.ApplyAngularImpulse(self.angularImpulseCoeff * \
                                      self.body.inertia * \
                                      -self.body.angularVelocity, wake=True)

        currForwardNormal = self.getForwardVelocity()
        currForwardSpeed = currForwardNormal.Normalize()
        dragForce = self.dragForceCoeff * currForwardSpeed
        self.body.ApplyForce(dragForce * currForwardNormal, \
                             self.body.worldCenter, wake=True)

    def updateDrive(self, linear_cmd, dt):
        currForwardNormal = self.body.GetWorldVector((0,1))
        currSpeed = Box2D.b2Dot(self.getForwardVelocity(), currForwardNormal)

        # Each tire should power itself and 1/4th of the car
        mass = self.body.mass + self.car_weight / 4
        delta_v = linear_cmd.x - currSpeed

        # Assume that the time step is 1 (box2d handles the conversion?)
        force = mass * delta_v

        # Accounts for friction
        force += self.dragForceCoeff * linear_cmd.x

        # Ensures that the engine/brakes are powerful enough
        if force > self.maxDriveForce:
            force = self.maxDriveForce
        elif force < -self.maxDriveForce:
            force = -self.maxDriveForce

        self.body.ApplyForce(force * currForwardNormal, \
                             self.body.worldCenter, wake=True)
