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

# locations used for accessing car position and orientation
JOINT_IDS = (1, 0, 2)  # X, Y, W
BASE_QUATERNION = [0., 0., 0.]


class Car(object):
    """Handles Car actions and instance based parameters."""

    def __init__(self, car_id, pos, orient, car_properties):
        """Sets instance-based properties for a car and generates instance-based properties for a sim run."""
        self._MAX_CURVATURE = None
        self._STEERING_RATE = None
        self._THROTTLE_TAU = None
        self._STEERING_THROW = None
        self._LENGTH = None
        self._MAX_SPEED = None
        self._psi = None
        self.cmd = None
        self.joint_ids = None
        self._v_rear = None
        self.id = car_id
        self.init_pos = None
        self.orient = None
        self.simulate_effort = car_properties['simulate_effort']
        self.set_properties(car_properties)

        self.body_link_id = 1 # urdf configuration

        self.reset(pos, orient)

    def set_properties(self, car_properties):
        self._LENGTH = car_properties['length']
        self._MAX_SPEED = car_properties['max_speed']
        self._THROTTLE_TAU = car_properties['throttle_tau']
        self._STEERING_THROW = car_properties['steering_throw']
        self._STEERING_RATE = car_properties['steering_rate']
        self._MAX_CURVATURE = math.tan(self._STEERING_THROW) / self._LENGTH

    def setCmd(self, cmd):
        self.cmd = cmd

    def step(self, dt):
        """
        Runs a simulation step of the car, moving it within the time step.
        @param dt: The duration of the car time step.
        """
        if self.cmd is None:
            return

        _, orient = self.get_pose()
        theta = p.getEulerFromQuaternion(orient)[2]

        if self.simulate_effort:
            # transform control input to reference angles and velocities
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

            # extrapolate future state sing bicycle model
            x_dot = self._v_rear * math.cos(theta + math.atan(math.tan(self._psi) / 2.0)) * \
                    math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
            y_dot = self._v_rear * math.sin(theta + math.atan(math.tan(self._psi) / 2.0)) * \
                    math.sqrt(math.pow(math.tan(self._psi), 2.0) / 4.0 + 1.0)
            omega = self._v_rear * math.tan(self._psi) / self._LENGTH
        else:
            body_vel = self.cmd[0]
            if abs(body_vel) > self._MAX_SPEED:
                body_vel = math.copysign(self._MAX_SPEED, body_vel)

            body_curve = self.cmd[1]
            if abs(body_curve) > self._MAX_CURVATURE:
                body_curve = math.copysign(self._MAX_CURVATURE, body_curve)

            x_dot = body_vel * math.cos(theta)
            y_dot = body_vel * math.sin(theta)
            omega = body_vel * body_curve

        p.setJointMotorControlArray(self.id, self.joint_ids,
                                    targetVelocities=(x_dot, y_dot, omega),
                                    controlMode=p.VELOCITY_CONTROL,
                                    forces=(5000, 5000, 5000))

    def get_pose(self, noise=None):
        """
        Randomizes and sets a new position for the car.
        @param noise: The sensor noise and if it is present (None=no noise).
        @return: The position and orientation of the car.
        """
        pos = p.getLinkState(self.id, self.body_link_id)[0]
        heading = p.getJointState(self.id, self.joint_ids[2])[0]
        orient = (0.0, 0.0, heading)
        if noise:
            if np.random.uniform() < noise['dropout']:
                return None, None
            else:
                pos = np.random.normal(pos, noise['pos'])
                orient = np.random.normal(orient, noise['orient'])
        print("position: pos:",pos, "quaternion",p.getQuaternionFromEuler(orient))
        return pos, p.getQuaternionFromEuler(orient)

    def get_velocity(self):
        """Returns the linear and angular velocity of the car."""

        link_state = p.getLinkState(self.id, self.body_link_id, computeLinkVelocity=1)
        orient = link_state[1]
        linear, angular = link_state[6:8]
        heading = p.getEulerFromQuaternion(orient)[2]
        r_inv = np.array([[math.cos(heading), -math.sin(heading), 0.],
                          [math.sin(heading), math.cos(heading), 0.],
                          [0., 0., 1.]], dtype=np.float)
        linear = r_inv @ linear
        print("velocity: linear:",linear, "angular",angular)
        return linear, angular

    def reset(self, pos, orient):
        """Resets the car state with the new pose and orient."""
        self.init_pos = pos
        self.orient = orient

        self._v_rear = 0.0
        self._psi = 0.0

        p.resetBasePositionAndOrientation(self.id, [0., 0., pos[2]], p.getQuaternionFromEuler(BASE_QUATERNION))
        print("reseting car to new pose and orient")
        self.joint_ids = JOINT_IDS  # X, Y, W
        p.resetJointState(self.id, self.joint_ids[0], targetValue=pos[0])
        p.resetJointState(self.id, self.joint_ids[1], targetValue=pos[1])
        p.resetJointState(self.id, self.joint_ids[2], targetValue=orient[2])

        self.cmd = None

    def check_overlap(self, pos):
        """
        Returns whether the position will overlap with the current car.
        @param pos: The position of the other object.
        @return: Boolean if the positions overlap (true = overlap).
        """
        print("x",pos[0],"y",pos[1])
        print("x1",self.init_pos[0],"y1",self.init_pos[1])
        val =((pos[0] - self.init_pos[0]) * (pos[0] - self.init_pos[0])) + ((pos[1] - self.init_pos[1]) * (pos[1] - self.init_pos[1]))
        print(val)
        dist = math.sqrt(val)
        return dist < self._LENGTH
