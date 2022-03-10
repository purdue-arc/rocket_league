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
    def __init__(self, carID, pos, orient, car_properties):
        self.id = carID

        # physical constants
        self._LENGTH = car_properties['length']
        self._MAX_SPEED = car_properties['max_speed']
        self._THROTTLE_TAU = car_properties['throttle_tau']
        self._STEERING_THROW = car_properties['steering_throw']
        self._STEERING_RATE = car_properties['steering_rate']

        # URDF Configuration
        self.body_link_id = 1

        # System state
        self._v_rear = 0.0
        self._psi = 0.0

        # Model configuration
        p.resetBasePositionAndOrientation(
            self.id, [0., 0., pos[2]], p.getQuaternionFromEuler([0., 0., 0.]))

        self.joint_ids = (1, 0, 2) # X, Y, W
        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])

    def step(self, cmd, dt):
        # tranfrom control input to reference angles and velocities
        v_rear_ref = cmd[0] * self._MAX_SPEED
        psi_ref = cmd[1] * self._STEERING_THROW

        # update rear wheel velocity using 1st order model
        self._v_rear = (self._v_rear - v_rear_ref) * math.exp(-dt/self._THROTTLE_TAU) + v_rear_ref

        # update steering angle using massless acceleration to a fixed rate
        if abs(psi_ref - self._psi) < self._STEERING_RATE * dt:
            self._psi = psi_ref
        else:
            if psi_ref > self._psi:
                self._psi += self._STEERING_RATE * dt
            else:
                self._psi -= self._STEERING_RATE * dt

        # get current yaw angle
        _, orient = self.getPose()
        theta = p.getEulerFromQuaternion(orient)[2]

        # using bicycle model, extrapolate future state
        x_dot = self._v_rear * math.cos(theta + math.atan(math.tan(self._psi) / 2.0)) * math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
        y_dot = self._v_rear * math.sin(theta + math.atan(math.tan(self._psi) / 2.0)) * math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
        omega = self._v_rear * math.tan(self._psi) / self._LENGTH

        p.setJointMotorControlArray(self.id, self.joint_ids,
            targetVelocities=(x_dot, y_dot, omega),
            controlMode=p.VELOCITY_CONTROL,
            forces=(5000, 5000, 5000))

    def getPose(self):
        pos = p.getLinkState(self.id, self.body_link_id)[0]
        heading = p.getJointState(self.id, self.joint_ids[2])[0]
        return (pos, p.getQuaternionFromEuler((0.0, 0.0, heading)))

    def getVelocity(self):
        link_state = p.getLinkState(self.id, self.body_link_id, computeLinkVelocity=1)
        orient = link_state[1]
        linear, angular = link_state[6:8]
        heading = p.getEulerFromQuaternion(orient)[2]
        r_inv = np.array([[math.cos(heading), -math.sin(heading), 0.],
                          [math.sin(heading), math.cos(heading), 0.],
                         [0., 0., 1.]], dtype=np.float)
        linear = r_inv @ linear
        return (linear, angular)

    def reset(self, pos, orient):
        self._v_rear = 0.0
        self._psi = 0.0

        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])
