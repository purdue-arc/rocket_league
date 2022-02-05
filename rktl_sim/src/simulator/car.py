"""Contains the Car class.
License:
  BSD 3-Clause License
  Copyright (c) 2021, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import pybullet as p
import math
import numpy as np
import time

class Car(object):
    def __init__(self, carID, length, pos, orient):
        self.id = carID
        self._steering_angle = 0.
        self._steering_limit = math.pi / 6.
        self._length = length
        self._length_r = self._length / 2.
        self._steering_rate = 2*self._steering_limit / 0.25
        self._velocity_coeff = 1.42

        # Collision handling
        self._collision_started = False
        self._collision_friction = 0.05

        # System response
        self._throttle_state = np.zeros((2,), dtype=np.float)
        self._A = np.array([[-8., -4.199], [8., 0.]])
        self._B = np.array([[2], [0]])
        self._C = np.array([[0, 12.6]])

        self._car_handle = p.createConstraint(
            self.id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])

        p.resetBasePositionAndOrientation(
            self.id, pos, p.getQuaternionFromEuler(orient))

        # Testing
        self.start_pos = None
        self.start_time = None

    def step(self, cmd, contact, dt):
        des_throttle = cmd[0]
        steering = cmd[1]

        # Handle collision
        if contact:
            if not self._collision_started:
                self._throttle_state = np.zeros((2,), dtype=np.float)
                self._collision_started = True
            else:
                des_throttle *= self._collision_friction
        else:
            self._collision_started = False

        # Compute 2nd-order response of throttle
        throttle = self._C @ self._throttle_state
        throttle_dt = self._A @ self._throttle_state + \
            self._B @ np.array([des_throttle])
        self._throttle_state += throttle_dt * dt

        # Compute 0th-order response of steering
        steering = max(min(steering, self._steering_limit), -
                       self._steering_limit)
        steering_dt = (steering - self._steering_angle) / dt
        steering_dt = max(
            min(steering_dt, self._steering_rate), -self._steering_rate)
        self._steering_angle += steering_dt * dt

        # Compute motion using bicycle model
        pos, orient = self.getPose()
        heading = p.getEulerFromQuaternion(orient)[2]

        beta = math.atan(
            (self._length_r) * math.tan(self._steering_angle) / self._length)
        x_vel = throttle * math.cos(heading + beta) * self._velocity_coeff
        y_vel = throttle * math.sin(heading + beta) * self._velocity_coeff
        w = throttle * math.tan(self._steering_angle) * \
            math.cos(beta) / self._length * self._velocity_coeff

        pos = (pos[0] + x_vel*dt, pos[1] + y_vel*dt, pos[2])
        orient = p.getQuaternionFromEuler([0., 0., heading + w * dt])
        orient = self.orientToGlobal(orient)

        if des_throttle != 0:
            time_diff = 0.0
            if self.start_time is None:
                self.start_time = time.time()
            else:
                time_diff = time.time() - self.start_time

            pos_diff = 0.0
            if self.start_pos is None:
                self.start_pos = pos[0][0]
            else:
                pos_diff = pos[0][0] - self.start_pos

            linear, _ = p.getBaseVelocity(self.id)
            print(time_diff, pos_diff, throttle[0], x_vel[0], linear[0])
        else:
            self.start_pos = None
            self.start_time = None

        # p.changeConstraint(self._car_handle, pos, orient, maxForce=450)
        p.setJointMotorControl2(self.id, 0,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity = x_vel,
            force = 500)

    def orientToLocal(self, orient):
        orient = p.getEulerFromQuaternion(orient)
        orient = (orient[0], orient[1], orient[2] - (math.pi / 2.))
        return p.getQuaternionFromEuler(orient)

    def orientToGlobal(self, orient):
        orient = p.getEulerFromQuaternion(orient)
        orient = (orient[0], orient[1], orient[2] + (math.pi / 2.))
        return p.getQuaternionFromEuler(orient)

    def getPose(self):
        position, orient = p.getBasePositionAndOrientation(self.id)
        orient = self.orientToLocal(orient)
        return (position, orient)

    def getVelocity(self):
        _, orientation = p.getBasePositionAndOrientation(self.id)
        heading = p.getEulerFromQuaternion(orientation)[2]
        r_inv = np.array([[math.cos(heading), -math.sin(heading), 0.],
                          [math.sin(heading), math.cos(heading), 0.],
                         [0., 0., 1.]], dtype=np.float)
        linear, angular = p.getBaseVelocity(self.id)
        linear = r_inv @ linear
        return (linear, angular)

    def reset(self, pos, orient):
        self._steering_angle = 0
        self._throttle_state = np.zeros((2,), dtype=np.float)

        p.resetBasePositionAndOrientation(
            self.id, pos, p.getQuaternionFromEuler(orient))
