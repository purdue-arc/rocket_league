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
            restitution=1.0)

        if 'ball' in field_setup:
            ballPos = field_setup['ball']
            self.initBallPos = ballPos
        else:
            ballPos = [random.uniform(spawn_bounds[0][0], spawn_bounds[0][1]),
                       random.uniform(spawn_bounds[1][0], spawn_bounds[1][1]),
                       random.uniform(spawn_bounds[2][0], spawn_bounds[2][1])]
            self.initBallPos = None
        self._ballID = p.loadURDF(
            urdf_paths["ball"], ballPos, zeroOrient)
        p.changeDynamics(bodyUniqueId=self._ballID,
            linkIndex=-1,
            restitution=0.7, 
            linearDamping=0, angularDamping=0,
            rollingFriction=0.0001, spinningFriction=0.001)

        # Initialize ball with some speed
        self._speed_bound = math.sqrt(2.) * speed_init 
        ballVel = [random.uniform(-self._speed_bound, self._speed_bound),
                   random.uniform(-self._speed_bound, self._speed_bound), 0.]
        p.resetBaseVelocity(self._ballID, ballVel, zeroOrient)

        self._goalAID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalA"], zeroOrient, useFixedBase=1
        )

        self._goalBID = p.loadURDF(
            urdf_paths["goal"], field_setup["goalB"], zeroOrient, useFixedBase=1
        )

        self._walls = {}  
        lSidewallID = p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["lsidewall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=lSidewallID,
            linkIndex=-1,
            restitution=1.0)
        self._walls[lSidewallID] = True

        rSidewallId = p.loadURDF(
            urdf_paths["sidewall"],
            field_setup["rsidewall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=rSidewallId,
            linkIndex=-1,
            restitution=1.0)
        self._walls[rSidewallId] = True

        # TODO: Improve handling of split walls
        flBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["flbackwall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=flBackwallID,
            linkIndex=-1,
            restitution=1.0)
        self._walls[flBackwallID] = True

        frBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["frbackwall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=frBackwallID,
            linkIndex=-1,
            restitution=1.0)
        self._walls[frBackwallID] = True

        blBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["blbackwall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=blBackwallID,
            linkIndex=-1,
            restitution=1.0)
        self._walls[blBackwallID] = True

        brBackwallID = p.loadURDF(
            urdf_paths["backwall"],
            field_setup["brbackwall"],
            zeroOrient, useFixedBase=1,
        )
        p.changeDynamics(bodyUniqueId=brBackwallID,
            linkIndex=-1,
            restitution=1.0)
        self._walls[brBackwallID] = True

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

        p.setPhysicsEngineParameter(useSplitImpulse=1, restitutionVelocityThreshold=0.0001)
        p.setGravity(0, 0, -10)

    def step(self, throttle_cmd, steering_cmd, dt):
        """Advance one time-step in the sim."""
        ballContacts = p.getContactPoints(bodyA=self._ballID)
        for contact in ballContacts:
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
            wallContact = False
            carContacts = p.getContactPoints(bodyA=car.id)
            for contact in carContacts:
                if contact[2] in self._walls:
                    wallContact = True
                    break
            car.step((throttle_cmd, steering_cmd), wallContact, dt)

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

        ballPos = self.initBallPos
        if ballPos is None:
            ballPos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                        random.uniform(
                self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1])]
        p.resetBasePositionAndOrientation(
            self._ballID, ballPos, p.getQuaternionFromEuler([0, 0, 0])
        )

        ballVel = [random.uniform(-self._speed_bound, self._speed_bound),
                   random.uniform(-self._speed_bound, self._speed_bound), 0.]
        p.resetBaseVelocity(self._ballID, ballVel, [0, 0, 0])

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
