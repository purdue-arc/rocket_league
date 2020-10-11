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

# 3rd party packages
import Box2D
from pygame import draw

maxForwardSpeed = 1
maxBackwardSpeed = -1
maxDriveForce = 0.0001
maxLateralImpulse = 0.03

class Tire:
    """Simulates a single tire of a vehicle"""
    def __init__(self, world, width, length, density, torque, 
                    maxForwardSpeed, maxBackwardSpeed, maxDriveForce, 
                    maxLateralImpulse, dragForceCoeff, anglularImpulseCoeff):
        self.bodyDef = Box2D.b2BodyDef()
        self.bodyDef.type = Box2D.b2_dynamicBody
        self.body = world.CreateBody(self.bodyDef)

        self.shape = Box2D.b2PolygonShape(box=(width/2, length/2))
        self.body.CreateFixture(shape=self.shape, density=density)

        self.torque = torque
        self.maxForwardSpeed = maxForwardSpeed
        self.maxBackwardSpeed = maxBackwardSpeed
        self.maxDriveForce = maxDriveForce
        self.maxLateralImpulse = maxLateralImpulse
        self.dragForceCoeff = dragForceCoeff
        self.anglularImpulseCoeff = anglularImpulseCoeff

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
        self.body.ApplyAngularImpulse(self.anglularImpulseCoeff * \
                                      self.body.inertia * \
                                      -self.body.angularVelocity, wake=True)

        currForwardNormal = self.getForwardVelocity()
        currForwardSpeed = currForwardNormal.Normalize()
        dragForce = self.dragForceCoeff * currForwardSpeed
        self.body.ApplyForce(dragForce * currForwardNormal, \
                             self.body.worldCenter, wake=True)

    def updateDrive(self, control):
        desiredSpeed = 0
        if control[0] == 1:
            desiredSpeed = self.maxForwardSpeed
        elif control[0] == -1:
            desiredSpeed = self.maxBackwardSpeed
        else:
            return

        currForwardNormal = self.body.GetWorldVector((0,1))
        currSpeed = Box2D.b2Dot(self.getForwardVelocity(), currForwardNormal)

        force = 0
        if desiredSpeed > currSpeed:
            force = self.maxDriveForce
        elif desiredSpeed < currSpeed:
            force = -self.maxDriveForce
        else:
            return

        self.body.ApplyForce(force * currForwardNormal, \
                             self.body.worldCenter, wake=True)

    def updateTurn(self, control):
        desiredTorque = 0
        if control[1] == 1:
            desiredTorque = self.torque
        elif control[1] == -1:
            desiredTorque = -self.torque
        else:
            return

        self.body.ApplyTorque(desiredTorque, wake=True)