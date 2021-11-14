#!/usr/bin/env python3

"""Contains the Sim class.

License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
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
import math
import os
import pybullet as p
import pybullet_data as p_data
import random

# Local modules
from simulator.car import Car


class Sim(object):
    """Oversees components of the simulator"""

    def __init__(self, urdf_paths, field_setup, spawn_bounds, render_enabled, field_length, goal_offset):
        if render_enabled:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        self.field_length = field_length
        self.goal_offset = goal_offset
        
        self.spawn_bounds = spawn_bounds
        p.setAdditionalSearchPath(p_data.getDataPath())
        self._planeID = p.loadURDF(urdf_paths["plane"])

        zeroOrient = p.getQuaternionFromEuler([0, 0, 0])

        randBallPos = [random.uniform(spawn_bounds[0][0], spawn_bounds[0][1]),
                       random.uniform(spawn_bounds[1][0], spawn_bounds[1][1]), 0.05]
        self._ballID = p.loadURDF(
            urdf_paths["ball"], randBallPos, zeroOrient)

        self._goalAID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalA"], zeroOrient, useFixedBase=1
        )

        self._goalBID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalB"], zeroOrient, useFixedBase=1
        )

        p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["lsidewall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["rsidewall"],
            zeroOrient,
            useFixedBase=1,
        )

        # TODO: Improve handling of split walls
        p.loadURDF(
            urdf_paths["backwall"],
            field_setup["flbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.loadURDF(
            urdf_paths["backwall"],
            field_setup["frbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.loadURDF(
            urdf_paths["backwall"],
            field_setup["blbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.loadURDF(
            urdf_paths["backwall"],
            field_setup["brbackwall"],
            zeroOrient,
            useFixedBase=1,
        )

        self._cars = {}

        randCarPos = [random.uniform(spawn_bounds[0][0], spawn_bounds[0][1]),
                      random.uniform(spawn_bounds[1][0], spawn_bounds[1][1]), 0.05]
        randCarOrient = [0, 0, random.uniform(0, 2 * math.pi)]
        self._carID = p.loadURDF(
            urdf_paths["car"], randCarPos, zeroOrient)
        self._cars[self._carID] = Car(
            self._carID, 0.5, randCarPos, randCarOrient,
        )

        self.touched_last = None
        self.scored = False
        self.running = True
        self.winner = None

        p.setGravity(0, 0, -10)

    def step(self, throttle_cmd, steering_cmd, dt):
        """Advance one time-step in the sim."""
        if self.running:
            contacts = p.getContactPoints(bodyA=self._ballID)
            for contact in contacts:
                if contact[2] in self._cars:
                    self.touchedLast = contact[2]
                elif contact[2] == self._goalAID:
                    self.scored = True
                    self.winner = "A"
                elif contact[2] == self._goalBID:
                    self.scored = True
                    self.winner = "B"

            for car in self._cars.values():
                car.step((throttle_cmd, steering_cmd), dt)

            p.stepSimulation()

    def getCarPose(self):
        # TODO: Provide translation from ARC IDs to Sim IDs
        return list(self._cars.values())[0].getPose()

    def getCarVelocity(self):
        # TODO: Provide translation from ARC IDs to Sim IDs
        return list(self._cars.values())[0].getVelocity()

    def getBallPose(self):
        pos, _ = p.getBasePositionAndOrientation(self._ballID)
        return pos, p.getQuaternionFromEuler([0, 0, 0])

    def getBallVelocity(self):
        return p.getBaseVelocity(self._ballID)

    def reset(self):
        self.running = False
        self.scored = False
        self.winner = None
        self.touched_last = None

        randBallPos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                       random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]), 0.125]
        p.resetBasePositionAndOrientation(
            self._ballID, randBallPos, p.getQuaternionFromEuler([0, 0, 0])
        )

        for car in self._cars.values():
            randCarPos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                          random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]), 0.125]
            randCarOrient = [0, 0, random.uniform(0, 2 * math.pi)]
            car.reset(randCarPos, randCarOrient)

        self.running = True
