"""Contains the Sim class.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import math
import pybullet as p
import random
import numpy as np

# Local modules
from simulator.car import Car

class Sim(object):
    """Oversees components of the simulator"""

    def __init__(self, urdf_paths, field_setup, spawn_bounds, render_enabled):
        if render_enabled:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        self.urdf_paths = urdf_paths
        self.spawn_bounds = spawn_bounds
    
        zeroOrient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        self._planeID = None
        if "plane" in urdf_paths:
            self._planeID = p.loadURDF(
                urdf_paths["plane"], [0, 0, 0], zeroOrient, useFixedBase=1
            )
            p.changeDynamics(bodyUniqueId=self._planeID, linkIndex=-1, restitution=1.0)

        self._goalAID = None
        self._goalBID = None
        if "goal" in urdf_paths:
            self._goalAID = p.loadURDF(
                urdf_paths["goal"], field_setup["goalA"], zeroOrient, useFixedBase=1
            )

            self._goalBID = p.loadURDF(
                urdf_paths["goal"], field_setup["goalB"], zeroOrient, useFixedBase=1
            )

        self._walls = {}
        if "sidewall" in urdf_paths:
            lSidewallID = p.loadURDF(
                urdf_paths["sidewall"],
                field_setup["lsidewall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=lSidewallID, linkIndex=-1, restitution=1.0)
            self._walls[lSidewallID] = True

            rSidewallId = p.loadURDF(
                urdf_paths["sidewall"],
                field_setup["rsidewall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=rSidewallId, linkIndex=-1, restitution=1.0)
            self._walls[rSidewallId] = True

        if "backwall" in urdf_paths:
            # TODO: Improve handling of split walls
            flBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["flbackwall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=flBackwallID, linkIndex=-1, restitution=1.0)
            self._walls[flBackwallID] = True

            frBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["frbackwall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=frBackwallID, linkIndex=-1, restitution=1.0)
            self._walls[frBackwallID] = True

            blBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["blbackwall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=blBackwallID, linkIndex=-1, restitution=1.0)
            self._walls[blBackwallID] = True

            brBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["brbackwall"],
                zeroOrient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=brBackwallID, linkIndex=-1, restitution=1.0)
            self._walls[brBackwallID] = True

        self._cars = {}
        self._ballID = None

        self.touched_last = None
        self.scored = False
        self.winner = None

        p.setPhysicsEngineParameter(useSplitImpulse=1, restitutionVelocityThreshold=0.0001)
        p.setGravity(0, 0, -10)

    def create_ball(self, urdf_name, init_pose=None, init_speed=None, noise=None):
        if urdf_name in self.urdf_paths:
            zeroOrient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
            if init_pose:
                ballPos = init_pose["pos"]
                self.initBallPos = ballPos
            else:
                ballPos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
                self.initBallPos = None
            self._ballID = p.loadURDF(
                self.urdf_paths[urdf_name], ballPos, zeroOrient)
            p.changeDynamics(
                bodyUniqueId=self._ballID,
                linkIndex=-1,
                restitution=0.7,
                linearDamping=0,
                angularDamping=0,
                rollingFriction=0.0001,
                spinningFriction=0.001,
            )

            # initize ball with some speed
            self._speed_bound = math.sqrt(2.0) * init_speed
            ballVel = [
                random.uniform(-self._speed_bound, self._speed_bound),
                random.uniform(-self._speed_bound, self._speed_bound),
                0.0,
            ]
            p.resetBaseVelocity(self._ballID, ballVel, zeroOrient)
            self.ball_noise = noise
            return self._ballID
        else:
            return None

    def create_car(self, urdf_name, init_pose=None, noise=None, props=None):
        if urdf_name in self.urdf_paths:
            zeroPos = [0.0, 0.0, 0.0]
            zeroOrient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
            self._carID = p.loadURDF(self.urdf_paths[urdf_name], zeroPos, zeroOrient)
            if init_pose:
                if "pos" in init_pose:
                    carPos = init_pose["pos"]
                else:
                    carPos = zeroPos

                if "orient" in init_pose:
                    carOrient = init_pose["orient"]
                else:
                    carOrient = zeroOrient

                self.initCarPos = carPos
                self.initCarOrient = carOrient
            else:
                carPos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
                carOrient = [0.0, 0.0, random.uniform(0, 2 * math.pi)]
                self.initCarPos = None
                self.initCarOrient = None
            self._cars[self._carID] = Car(
                self._carID,
                carPos,
                carOrient,
                props
            )
            self.car_noise = noise
            return self._carID
        else:
            return None

    def step(self, car_cmd, dt):
        """Advance one time-step in the sim."""
        if self._ballID is not None:
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
        p_dt = 1.0 / 240.0
        for _ in range(round(dt / p_dt)):
            # Step kinematic objects independently, at max possible rate
            for car in self._cars.values():
                car.step(car_cmd, p_dt)
            p.stepSimulation()

    def getCarPose(self):
        cars = list(self._cars.values())
        if len(cars) == 0:
            return None

        # TODO: Provide translation from ARC IDs to Sim IDs
        return cars[0].getPose(noise=self.car_noise)

    def getCarVelocity(self):
        cars = list(self._cars.values())
        if len(cars) == 0:
            return None

        # TODO: Provide translation from ARC IDs to Sim IDs
        return cars[0].getVelocity()

    def getBallPose(self):
        if self._ballID is None:
            return None
        pos, _ = p.getBasePositionAndOrientation(self._ballID)
        
        if self.ball_noise:
            pos = np.random.normal(
                pos, self.ball_noise['pos'])
        return pos, p.getQuaternionFromEuler([0, 0, 0])

    def getBallVelocity(self):
        if self._ballID is None:
            return None
        return p.getBaseVelocity(self._ballID)

    def reset(self):
        self.scored = False
        self.winner = None
        self.touched_last = None

        if self._ballID is not None:
            ballPos = self.initBallPos
            if ballPos is None:
                ballPos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
            p.resetBasePositionAndOrientation(
                self._ballID, ballPos, p.getQuaternionFromEuler([0, 0, 0])
            )

            ballVel = [
                random.uniform(-self._speed_bound, self._speed_bound),
                random.uniform(-self._speed_bound, self._speed_bound),
                0.0,
            ]
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
