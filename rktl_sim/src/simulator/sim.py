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

    class NoURDFError(Exception):
        """Exception for when a URDF isn't provided"""
        pass

    def __init__(self, props, urdf_paths, spawn_bounds, render_enabled):
        if render_enabled:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        self.props = props
        self.urdf_paths = urdf_paths
        self.spawn_bounds = spawn_bounds

        zero_pos = [0.0, 0.0, 0.0]
        zero_orient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        self._plane_id = None
        if "plane" in urdf_paths:
            self._plane_id = p.loadURDF(
                urdf_paths["plane"], zero_pos, zero_orient, useFixedBase=1
            )
            print(self.props)
            self.configure_dynamics(self._plane_id, "floor")
        else:
            raise self.NoURDFError()

        if "walls" in urdf_paths:
            self._walls_id = p.loadURDF(
                urdf_paths["walls"], zero_pos, zero_orient, useFixedBase=1
            )
            self.configure_dynamics(self._walls_id, "walls")
        else:
            raise self.NoURDFError()

        self._goal_a_id = None
        self._goal_b_id = None
        if "goal_a" in urdf_paths and "goal_b" in urdf_paths:
            self._goal_a_id = p.loadURDF(
                urdf_paths["goal_a"], zero_pos, zero_orient, useFixedBase=1
            )

            self._goal_b_id = p.loadURDF(
                urdf_paths["goal_b"], zero_pos, zero_orient, useFixedBase=1
            )
        else:
            return self.NoURDFError()

        self._cars = {}
        self._car_data = {}
        self._ball_id = None

        self.touched_last = None
        self.scored = False
        self.winner = None

        if 'engine' in self.props and self.props['engine'] is not None:
            p.setPhysicsEngineParameter(**self.props['engine'])
        p.setGravity(0, 0, -10)

    def configure_dynamics(self, body_id, body_type):
        if 'dynamics' not in self.props or \
            self.props['dynamics'] is None or \
            body_type not in self.props['dynamics']:
            return

        num_links = p.getNumJoints(body_id)
        for i in range(num_links):
            p.changeDynamics(body_id, i, **self.props['dynamics'][body_type])

    def create_ball(self, urdf_name, init_pose=None, init_speed=None,
        noise=None, init_vel=None):
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
            self._ball_id = p.loadURDF(
                self.urdf_paths[urdf_name], ball_pos, zero_orient)
            self.configure_dynamics(self._ball_id, "ball")

            # initize ball with some speed
            if init_vel:
                ball_vel = init_vel
            else:
                if init_speed:
                    self._speed_bound = math.sqrt(2.0) * init_speed
                else:
                    self._speed_bound = 0.0
                ball_vel = [
                    random.uniform(-self._speed_bound, self._speed_bound),
                    random.uniform(-self._speed_bound, self._speed_bound),
                    0.0,
                ]
            p.resetBaseVelocity(self._ball_id, ball_vel, zero_orient)
            self.ball_noise = noise
            return self._ball_id
        else:
            return None

    def create_car(self, urdf_name, init_pose=None, noise=None, car_props=None):
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
                car_props
            )
            self.configure_dynamics(car_id, "car")

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
        if self._ball_id is not None:
            ball_contacts = p.getContactPoints(bodyA=self._ball_id)
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
        if self._ball_id is None:
            return None
        pos, _ = p.getBasePositionAndOrientation(self._ball_id)

        if add_noise and self.ball_noise:
            pos = np.random.normal(
                pos, self.ball_noise['pos'])
        return pos, p.getQuaternionFromEuler([0, 0, 0])

    def get_ball_velocity(self):
        if self._ball_id is None:
            return None
        return p.getBaseVelocity(self._ball_id)

    def reset(self):
        self.scored = False
        self.winner = None
        self.touched_last = None

        if self._ball_id is not None:
            ball_pos = self.init_ball_pos
            if ball_pos is None:
                ball_pos = [
                    random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
            p.resetBasePositionAndOrientation(
                self._ball_id, ball_pos, p.getQuaternionFromEuler([0, 0, 0])
            )

            ball_vel = [
                random.uniform(-self._speed_bound, self._speed_bound),
                random.uniform(-self._speed_bound, self._speed_bound),
                0.0,
            ]
            p.resetBaseVelocity(self._ball_id, ball_vel, [0, 0, 0])

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
