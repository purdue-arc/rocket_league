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

JOINT_IDS = (1, 0, 2)  # X, Y, W
BASE_QUATERNION = [0., 0., 0.]

class Car(object):
    def __init__(self, car_id, pos, orient, car_properties):
        self._psi = None
        self.cmd = None
        self.joint_ids = None
        self._v_rear = None
        self._STEERING_RATE = None
        self._MAX_CURVATURE = None
        self._STEERING_THROW = None
        self._THROTTLE_TAU = None
        self._MAX_SPEED = None
        self._LENGTH = None
        self.id = car_id
        self.init_pos = None
        self.orient = None
        self.simulate_effort = car_properties['simulate_effort']

        # physical constants
        self.set_properties(car_properties)

        # urdf configuration
        self.body_link_id = 1

        self.reset(pos, orient)

    def set_properties(self, car_properties):
        """
        sets the physical car properties for the car
        @param car_properties: the general properties of the car that are set
        """
        self._LENGTH = car_properties['length']
        self._MAX_SPEED = car_properties['max_speed']
        self._THROTTLE_TAU = car_properties['throttle_tau']
        self._STEERING_THROW = car_properties['steering_throw']
        self._STEERING_RATE = car_properties['steering_rate']
        self._MAX_CURVATURE = math.tan(self._STEERING_THROW) / self._LENGTH

    def setCmd(self, cmd):
        self.cmd = cmd

    def step(self, dt):
        if self.cmd is None:
            return

        # get current yaw angle
        _, orient = self.get_pose()
        theta = p.getEulerFromQuaternion(orient)[2]

        if self.simulate_effort:
            # transfrom control input to reference angles and velocities
            v_rear_ref = self.cmd[0] * self._MAX_SPEED
            psi_ref = self.cmd[1] * self._STEERING_THROW

            # update rear wheel velocity using 1st order model
            self._v_rear = (self._v_rear - v_rear_ref) * math.exp(-dt / self._THROTTLE_TAU) + v_rear_ref

            # update steering angle using massless acceleration to a fixed rate
            if abs(psi_ref - self._psi) < self._STEERING_RATE * dt:
                self._psi = psi_ref
            else:
                if psi_ref > self._psi:
                    self._psi += self._STEERING_RATE * dt
                else:
                    self._psi -= self._STEERING_RATE * dt

            # using bicycle model, extrapolate future state
            x_dot = self._v_rear * math.cos(theta + math.atan(math.tan(self._psi) / 2.0)) * \
                    math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
            y_dot = self._v_rear * math.sin(theta + math.atan(math.tan(self._psi) / 2.0)) * \
                    math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
            omega = self._v_rear * math.tan(self._psi) / self._LENGTH
        else:
            body_vel = self.cmd[0]
            if abs(body_vel) > self._MAX_SPEED:
                body_vel = math.copysign(self._MAX_SPEED, body_vel)

            body_curv = self.cmd[1]
            if abs(body_curv) > self._MAX_CURVATURE:
                body_curv = math.copysign(self._MAX_CURVATURE, body_curv)

            x_dot = body_vel * math.cos(theta)
            y_dot = body_vel * math.sin(theta)
            omega = body_vel * body_curv

        p.setJointMotorControlArray(self.id, self.joint_ids,
                                    targetVelocities=(x_dot, y_dot, omega),
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=(5000, 5000, 5000))

    def get_pose(self, noise=None):
        pos = p.getLinkState(self.id, self.body_link_id)[0]
        heading = p.getJointState(self.id, self.joint_ids[2])[0]
        orient = (0.0, 0.0, heading)
        if noise:
            if np.random.uniform() < noise['dropout']:
                return None, None
            else:
                pos = np.random.normal(pos, noise['pos'])
                orient = np.random.normal(orient, noise['orient'])

        return pos, p.getQuaternionFromEuler(orient)

    def get_velocity(self):
        link_state = p.getLinkState(self.id, self.body_link_id, computeLinkVelocity=1)
        orient = link_state[1]
        linear, angular = link_state[6:8]
        heading = p.getEulerFromQuaternion(orient)[2]
        r_inv = np.array([[math.cos(heading), -math.sin(heading), 0.],
                          [math.sin(heading), math.cos(heading), 0.],
                          [0., 0., 1.]], dtype=np.float)
        linear = r_inv @ linear
        return linear, angular

    def reset(self, pos, orient):
        """
        resets the system state and the model configuration for the car
        uses the position and the orientation to reset the cars position
        @param pos: the new position of the car
        @param orient: the new orientation of the car
        """
        # save the car pos and orient:
        self.init_pos = pos
        self.orient = orient

        # system state
        self._v_rear = 0.0
        self._psi = 0.0

        # model configuration
        p.resetBasePositionAndOrientation(
            self.id, [0., 0., pos[2]], p.getQuaternionFromEuler(BASE_QUATERNION))

        self.joint_ids = JOINT_IDS  # X, Y, W
        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])

        self.cmd = None

    def check_overlap(self, pos):
        """
        returns whether the two positions of the car will overlap
        @param pos: the position of the other car
        @return: bolean if they overlap (true = overlap)
        """
        dist = math.sqrt(
            (pos[0] - self.init_pos[0]) * (pos[0] - self.init_pos[0]) + (pos[0] - self.init_pos[1]) * (pos[1] - self.init_pos[1]))
        return dist < self._LENGTH
