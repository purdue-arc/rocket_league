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

        zero_orient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        self._plane_id = None
        if "plane" in urdf_paths:
            self._plane_id = p.loadURDF(
                urdf_paths["plane"], [0, 0, 0], zero_orient, useFixedBase=1
            )
            p.changeDynamics(bodyUniqueId=self._plane_id, linkIndex=-1, restitution=1.0)

        if "walls" in urdf_paths:
            self._plane_id = p.loadURDF(
                urdf_paths["walls"], [0, 0, 0], zero_orient, useFixedBase=1
            )

        self._goal_a_id = None
        self._goal_b_id = None
        if "goal" in urdf_paths:
            self._goal_a_id = p.loadURDF(
                urdf_paths["goal"], field_setup["goalA"], zero_orient, useFixedBase=1
            )

            self._goal_b_id = p.loadURDF(
                urdf_paths["goal"], field_setup["goalB"], zero_orient, useFixedBase=1
            )

        self._cars = {}
        self._car_data = {}
        self._ballID = None

        self.touched_last = None
        self.scored = False
        self.winner = None

        p.setPhysicsEngineParameter(useSplitImpulse=1, restitutionVelocityThreshold=0.0001)
        p.setGravity(0, 0, -10)

    def create_ball(self, urdf_name, init_pose=None, init_speed=None, noise=None):
        if urdf_name in self.urdf_paths:
            zero_orient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
            if init_pose:
                ball_pos = init_pose["pos"]
                self.init_ball_pos = ball_pos
            else:
                ball_pos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
                self.init_ball_pos = None
            self._ballID = p.loadURDF(
                self.urdf_paths[urdf_name], ball_pos, zero_orient)
            p.changeDynamics(
                bodyUniqueId=self._ballID,
                linkIndex=-1,
                mass=0.200,
                lateralFriction=0.4,
                spinningFriction=0.001,
                rollingFriction=0.0001,
                restitution=0.7,
                linearDamping=0,
                angularDamping=0,
            )

            # initize ball with some speed
            self._speed_bound = math.sqrt(2.0) * init_speed
            ball_vel = [
                random.uniform(-self._speed_bound, self._speed_bound),
                random.uniform(-self._speed_bound, self._speed_bound),
                0.0,
            ]
            p.resetBaseVelocity(self._ballID, ball_vel, zero_orient)
            self.ball_noise = noise
            return self._ballID
        else:
            return None

    def create_car(self, urdf_name, init_pose=None, noise=None, props=None):
        if urdf_name in self.urdf_paths:
            zero_pos = [0.0, 0.0, 0.0]
            zero_orient = [0.0, 0.0, 0.0]
            car_id = p.loadURDF(self.urdf_paths[urdf_name], zero_pos, 
                p.getQuaternionFromEuler(zero_orient))
            if init_pose:
                if "pos" in init_pose:
                    car_pos = init_pose["pos"]
                else:
                    car_pos = zero_pos

                if "orient" in init_pose:
                    car_orient = init_pose["orient"]
                else:
                    car_orient = zero_orient

                init_car_pos = car_pos
                init_car_orient = car_orient
            else:
                car_pos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
                car_orient = [0.0, 0.0, random.uniform(0, 2 * math.pi)]
                init_car_pos = None
                init_car_orient = None

            self._cars[car_id] = Car(
                car_id,
                car_pos,
                car_orient,
                props
            )
            self._car_data[car_id] = {
                "init_pos": init_car_pos,
                "init_orient": init_car_orient,
                "noise": noise,
            }
            return car_id
        else:
            return None

    def delete_car(self, car_id):
        if car_id not in self._cars:
            return False

        p.removeBody(car_id)
        del self._cars[car_id]
        del self._car_data[car_id]
        return True

    def step(self, dt):
        """Advance one time-step in the sim."""
        if self._ballID is not None:
            ball_contacts = p.getContactPoints(bodyA=self._ballID)
            for contact in ball_contacts:
                if contact[2] in self._cars:
                    self.touchedLast = contact[2]
                elif contact[2] == self._goal_a_id:
                    self.scored = True
                    self.winner = "A"
                elif contact[2] == self._goal_b_id:
                    self.scored = True
                    self.winner = "B"

        # PyBullet steps at 240hz
        p_dt = 1.0 / 240.0
        for _ in range(round(dt / p_dt)):
            # Step kinematic objects independently, at max possible rate
            for car in self._cars.values():
                car.step(p_dt)
            p.stepSimulation()

    def get_car_pose(self, id, add_noise=False):
        if id not in self._cars:
            return None

        noise = self._car_data[id]['noise']
        if add_noise:
            return self._cars[id].get_pose(noise=noise)
        else:
            return self._cars[id].get_pose(noise=None)

    def get_car_velocity(self, id):
        if id not in self._cars:
            return None

        return self._cars[id].get_velocity()

    def set_car_command(self, id, cmd):
        if id not in self._cars:
            return None

        return self._cars[id].setCmd(cmd)

    def get_ball_pose(self, add_noise=False):
        if self._ballID is None:
            return None
        pos, _ = p.getBasePositionAndOrientation(self._ballID)

        if add_noise and self.ball_noise:
            pos = np.random.normal(
                pos, self.ball_noise['pos'])
        return pos, p.getQuaternionFromEuler([0, 0, 0])

    def get_ball_velocity(self):
        if self._ballID is None:
            return None
        return p.getBaseVelocity(self._ballID)

    def reset(self):
        self.scored = False
        self.winner = None
        self.touched_last = None

        if self._ballID is not None:
            ball_pos = self.init_ball_pos
            if ball_pos is None:
                ball_pos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
            p.resetBasePositionAndOrientation(
                self._ballID, ball_pos, p.getQuaternionFromEuler([0, 0, 0])
            )

            ball_vel = [
                random.uniform(-self._speed_bound, self._speed_bound),
                random.uniform(-self._speed_bound, self._speed_bound),
                0.0,
            ]
            p.resetBaseVelocity(self._ballID, ball_vel, [0, 0, 0])

        for car in self._cars.values():
            car_pos = self._car_data[car.id]["init_pos"]
            car_orient = self._car_data[car.id]["init_orient"]

            if car_pos is None:
                car_pos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                          random.uniform(
                              self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                          random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1])]

            if car_orient is None:
                car_orient = [0, 0, random.uniform(0, 2 * math.pi)]

            car.reset(car_pos, car_orient)
