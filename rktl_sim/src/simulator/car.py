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

        # URDF Configuration
        self.body_link_id = 1

        # Collision handling
        self._collision_started = False
        self._collision_friction = 0.05

        # System response
        self._throttle_state = np.zeros((2,), dtype=np.float)
        self._A = np.array([[-8., -4.199], [8., 0.]])
        self._B = np.array([[2], [0]])
        self._C = np.array([[0, 12.6]])

        # Model configuration
        p.resetBasePositionAndOrientation(
            self.id, [0., 0., pos[2]], p.getQuaternionFromEuler([0., 0., 0.]))

        self.y_joint_id = 0
        self.x_joint_id = 1
        self.w_joint_id = 2

        p.resetJointState(self.id, self.x_joint_id, targetValue=pos[1])
        p.resetJointState(self.id, self.y_joint_id, targetValue=pos[0])
        p.resetJointState(self.id, self.w_joint_id, targetValue=orient[2])

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
        x_vel = throttle * math.cos(heading + beta)
        y_vel = throttle * math.sin(heading + beta)
        w = throttle * math.tan(self._steering_angle) * \
            math.cos(beta) / self._length

        p.setJointMotorControl2(self.id, self.x_joint_id,
            controlMode=p.VELOCITY_CONTROL, targetVelocity=-x_vel, force=5000)
        p.setJointMotorControl2(self.id, self.y_joint_id,
            controlMode=p.VELOCITY_CONTROL, targetVelocity=-y_vel, force=5000)
        p.setJointMotorControl2(self.id, self.w_joint_id,
            controlMode=p.VELOCITY_CONTROL, targetVelocity=w, force=5000)

    def orientToLocal(self, orient):
        orient = p.getEulerFromQuaternion(orient)
        orient = (orient[0], orient[1], orient[2] - (math.pi / 2.))
        return p.getQuaternionFromEuler(orient)

    def orientToGlobal(self, orient):
        orient = p.getEulerFromQuaternion(orient)
        orient = (orient[0], orient[1], orient[2] + (math.pi / 2.))
        return p.getQuaternionFromEuler(orient)

    def getPose(self):
        pos, orient = p.getLinkState(self.id, self.body_link_id)[0:2]
        orient = self.orientToLocal(orient)
        return (pos, orient)

    def getVelocity(self):
        link_state = p.getLinkState(self.id, self.body_link_id,
            computeLinkVelocity=1)
        orient = link_state[1]
        linear, angular = link_state[6:8]
        heading = p.getEulerFromQuaternion(orient)[2]
        r_inv = np.array([[math.cos(heading), -math.sin(heading), 0.],
                          [math.sin(heading), math.cos(heading), 0.],
                         [0., 0., 1.]], dtype=np.float)
        linear = r_inv @ linear
        return (linear, angular)

    def reset(self, pos, orient):
        self._steering_angle = 0
        self._throttle_state = np.zeros((2,), dtype=np.float)

        p.resetJointState(self.id, self.x_joint_id, targetValue=pos[1])
        p.resetJointState(self.id, self.y_joint_id, targetValue=pos[0])
        p.resetJointState(self.id, self.w_joint_id, targetValue=orient[2])
