"""Contains the Sim class.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import math
import os
import pybullet as p
import random

# Local modules
from simulator.car import Car


class Sim(object):
    """Oversees components of the simulator"""

    def __init__(self, urdf_paths, field_setup, spawn_bounds, speed_init, render_enabled, field_length):
        if render_enabled:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        self.field_length = field_length

        self.spawn_bounds = spawn_bounds

        zeroOrient = p.getQuaternionFromEuler([0, 0, 0])
        self._planeID = p.loadURDF(urdf_paths["plane"], [0, 0, 0],
            zeroOrient, useFixedBase=1)
        p.changeDynamics(bodyUniqueId=self._planeID,
            linkIndex=-1,
            restitution=0.9)

        if 'ball' in field_setup:
            ballPos = field_setup['ball']
        else:
            ballPos = [random.uniform(spawn_bounds[0][0], spawn_bounds[0][1]),
                       random.uniform(spawn_bounds[1][0], spawn_bounds[1][1]),
                       random.uniform(spawn_bounds[2][0], spawn_bounds[2][1])]
        self._ballID = p.loadURDF(
            urdf_paths["ball"], ballPos, zeroOrient)
        p.changeDynamics(bodyUniqueId=self._ballID,
            linkIndex=-1,
            restitution=0.775)

        self._goalAID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalA"], zeroOrient, useFixedBase=1
        )

        self._goalBID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalB"], zeroOrient, useFixedBase=1
        )

        lSidewallID = p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["lsidewall"],
            zeroOrient,
        )
        p.changeDynamics(bodyUniqueId=lSidewallID,
            linkIndex=-1,
            restitution=0.9)

        rSidewallId = p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["rsidewall"],
            zeroOrient,
        )
        p.changeDynamics(bodyUniqueId=rSidewallId,
            linkIndex=-1,
            restitution=1.0)

        # TODO: Improve handling of split walls
        flBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["flbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=flBackwallID,
            linkIndex=-1,
            restitution=0.9)

        frBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["frbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=frBackwallID,
            linkIndex=-1,
            restitution=0.9)

        blBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["blbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=blBackwallID,
            linkIndex=-1,
            restitution=0.9)

        brBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["brbackwall"],
            zeroOrient,
            useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=brBackwallID,
            linkIndex=-1,
            restitution=0.9)

        self._cars = {}
        if 'car' in field_setup:
            carPos = field_setup['car']['pos']
            carOrient = field_setup['car']['orient']
            self.initCarPos = carPos
            self.initCarOrient = carOrient
        else:
            carPos = [random.uniform(spawn_bounds[0][0], spawn_bounds[0][1]),
                      random.uniform(spawn_bounds[1][0], spawn_bounds[1][1]),
                      random.uniform(spawn_bounds[2][0], spawn_bounds[2][1])]
            carOrient = [0., 0., random.uniform(0, 2 * math.pi)]
            self.initCarPos = None
            self.initCarOrient = None
        self._carID = p.loadURDF(
            urdf_paths["car"], carPos, p.getQuaternionFromEuler(carOrient))
        self._cars[self._carID] = Car(
            self._carID, 0.5, carPos, carOrient,
        )

        self.touched_last = None
        self.scored = False
        self.winner = None

        p.setPhysicsEngineParameter(restitutionVelocityThreshold=0.0)
        p.setGravity(0, 0, -10)
        
        # Initialize ball with some speed
        speed_bound = math.sqrt(2.) * speed_init 
        ballVel = [random.uniform(-speed_bound, speed_bound),
                   random.uniform(-speed_bound, speed_bound), 0.]
        p.resetBaseVelocity(self._ballID, ballVel, zeroOrient)

    def step(self, throttle_cmd, steering_cmd, dt):
        """Advance one time-step in the sim."""
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

        # PyBullet steps at 240hz
        for car in self._cars.values():
            car.step((throttle_cmd, steering_cmd), dt)

        for _ in range(int(dt * 240.)):
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
        self.scored = False
        self.winner = None
        self.touched_last = None

        randBallPos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                       random.uniform(
            self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
            random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1])]
        p.resetBasePositionAndOrientation(
            self._ballID, randBallPos, p.getQuaternionFromEuler([0, 0, 0])
        )

        for car in self._cars.values():
            carPos = self.initCarPos
            carOrient = self.initCarOrient

            if carPos is None:
                carPos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                          random.uniform(
                              self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                          random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1])]

            if carOrient is None:
                carOrient = [0, 0, random.uniform(0, 2 * math.pi)]

            car.reset(carPos, carOrient)
