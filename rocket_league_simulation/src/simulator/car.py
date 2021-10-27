#!/usr/bin/env python3

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
import pybullet as p
import math
import numpy as np


class Car(object):
    def __init__(self, carID, length, pos, orient):
        self.id = carID
        self._steering_angle = 0
        self._length = length

        # Init settings
        self._initPos = pos
        self._initOrient = orient

        # System response
        self._throttle_state = np.zeros((2,), dtype=np.float)
        self._A = np.array([[-8., -4.199], [8., 0.]])
        self._B = np.array([[2], [0]])
        self._C = np.array([[0, 2.1]])

        self._car_handle = p.createConstraint(
            self.id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])

        p.resetBasePositionAndOrientation(
            self.id, self._initPos, p.getQuaternionFromEuler(self._initOrient))

    def step(self, cmd, dt):
        des_throttle = cmd[0]
        steering = cmd[1]

        # Compute 2nd-order response of throttle
        throttle = self._C @ self._throttle_state
        throttle_dt = self._A @ self._throttle_state + \
            self._B @ np.array([des_throttle])
        self._throttle_state += throttle_dt * dt

        # Compute motion using bicycle model
        self._steering_angle += steering*dt
        pos, orient = p.getBasePositionAndOrientation(self.id)
        heading = p.getEulerFromQuaternion(orient)[2]
        x_vel = throttle * math.cos(heading + steering/2)
        y_vel = throttle * math.sin(heading + steering/2)
        w = (throttle * math.tan(heading + steering) *
             math.cos(heading + steering/2))/self._length

        pos = (pos[0] + x_vel*dt, pos[1] + y_vel*dt, pos[2])
        orientation = p.getQuaternionFromEuler([0, 0, heading + w * dt])

        p.changeConstraint(self._car_handle, pos, orientation)

    def getPose(self):
        return p.getBasePositionAndOrientation(self.id)

    def getVelocity(self):
        return p.getBaseVelocity(self.id)

    def reset(self):
        self._steering_angle = 0
        self._throttle_state = np.zeros((2,), dtype=np.float)

        p.resetBasePositionAndOrientation(
            self.id, self._initPos, p.getQuaternionFromEuler(self._initOrient))
