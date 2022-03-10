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

class Car(object):
    def __init__(self, carID, length, pos, orient):
        self.id = carID
        self._steering_angle = 0.
        self._steering_limit = math.pi / 6.
        self._length = length
        self._length_r = self._length / 2.
        self._steering_rate = 2*self._steering_limit / 0.25

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

        self.joint_ids = (1, 0, 2) # X, Y, W
        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])

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
        throttle = (self._C @ self._throttle_state)[0]
        throttle_dt = self._A @ self._throttle_state + self._B @ np.array([des_throttle])
        self._throttle_state += throttle_dt * dt

        # Compute 0th-order response of steering
        steering = max(min(steering, self._steering_limit), -
                       self._steering_limit)
        steering_dt = (steering - self._steering_angle) / dt
        steering_dt = max(
            min(steering_dt, self._steering_rate), -self._steering_rate)
        self._steering_angle += steering_dt * dt

        # Compute motion using bicycle model
        _, orient = self.getPose()
        heading = p.getEulerFromQuaternion(orient)[2]

        beta = math.atan(
            (self._length_r) * math.tan(self._steering_angle) / self._length)
        x_vel = throttle * math.cos(heading + beta)
        y_vel = throttle * math.sin(heading + beta)
        w = throttle * math.tan(self._steering_angle) * \
            math.cos(beta) / self._length

        p.setJointMotorControlArray(self.id, self.joint_ids,
            targetVelocities=(x_vel, y_vel, w),
            controlMode=p.VELOCITY_CONTROL, forces=(5000, 5000, 5000))


    def getPose(self):
        pos = p.getLinkState(self.id, self.body_link_id)[0]
        heading = p.getJointState(self.id, self.joint_ids[2])[0]
        return (pos, p.getQuaternionFromEuler((0.0, 0.0, heading)))

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

        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])
