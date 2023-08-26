"""Contains the Sim class.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import math
import re
import pybullet as p
import random
import numpy as np

# Local modules
from simulator.car import Car


class Sim(object):
    """
    Oversees instance-based parameters and objects of the simulator.
    Cars, ball objects, goal position, etc.
    """

    class NoURDFError(Exception):
        pass

    def __init__(self, props, urdf_paths, spawn_bounds, field_setup, render_enabled):
        """
        Initializes the playing field, field properties, and field elements.
        @param props: Connect the pybullet object based on the gui and direct.
        @param urdf_paths: Configure: filed type, walls, floor, goal a and b.
        @param spawn_bounds: Initialize cars list and other data related to them.
        @param render_enabled: Use the loadURDF via p.loadURDF (loads the specific instruction).
        """
        self.touchedLast = None
        self.ball_noise = None
        self._speed_bound = None
        self.init_ball_pos = None
        # determine if you want to display the simulation
        if render_enabled:
            self._client = p.connect(p.GUI)
        else:
            self._client = p.connect(p.DIRECT)

        self.props = props

        # urdf is used to encode kinetics and location of the object
        # use it for setting the field type, walls, floor and the different goals
        self.urdf_paths = urdf_paths
        self.spawn_bounds = spawn_bounds
        # set the floor for the simulation
        zero_pos = [0.0, 0.0, 0.0]
        zero_orient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
        self._plane_id = None

        # set the goals for the simulation
        self._goal_a_id = None
        self._goal_b_id = None
        if "plane" in urdf_paths:
            self._planeID = p.loadURDF(
                urdf_paths["plane"], [0, 0, 0], zero_orient, useFixedBase=1
            )
            p.changeDynamics(bodyUniqueId=self._planeID,
                             linkIndex=-1, restitution=1.0)

        self._goalAID = None
        self._goalBID = None
        if "goal" in urdf_paths:
            self._goalAID = p.loadURDF(
                urdf_paths["goal"], field_setup["goalA"], zero_orient, useFixedBase=1
            )

            self._goalBID = p.loadURDF(
                urdf_paths["goal"], field_setup["goalB"], zero_orient, useFixedBase=1
            )

        self._walls = {}
        if "sidewall" in urdf_paths:
            lSidewallID = p.loadURDF(
                urdf_paths["sidewall"],
                field_setup["lsidewall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=lSidewallID,
                             linkIndex=-1, restitution=1.0)
            self._walls[lSidewallID] = True

            rSidewallId = p.loadURDF(
                urdf_paths["sidewall"],
                field_setup["rsidewall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=rSidewallId,
                             linkIndex=-1, restitution=1.0)
            self._walls[rSidewallId] = True

        if "backwall" in urdf_paths:
            # TODO: Improve handling of split walls
            flBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["flbackwall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=flBackwallID,
                             linkIndex=-1, restitution=1.0)
            self._walls[flBackwallID] = True

            frBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["frbackwall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=frBackwallID,
                             linkIndex=-1, restitution=1.0)
            self._walls[frBackwallID] = True

            blBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["blbackwall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=blBackwallID,
                             linkIndex=-1, restitution=1.0)
            self._walls[blBackwallID] = True

            brBackwallID = p.loadURDF(
                urdf_paths["backwall"],
                field_setup["brbackwall"],
                zero_orient,
                useFixedBase=1,
            )
            p.changeDynamics(bodyUniqueId=brBackwallID,
                             linkIndex=-1, restitution=1.0)
            self._walls[brBackwallID] = True

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
        """
        Set the car's curvature and general car behavior.
        @param body_id: The id of the object to be configured.
        @param body_type: The specific type of object (ie ball,car,goal,etc).
        @return: Error if not initialized.
        """
        if 'dynamics' not in self.props or \
                self.props['dynamics'] is None or \
                body_type not in self.props['dynamics']:
            return

        num_links = p.getNumJoints(body_id)
        for i in range(num_links):
            p.changeDynamics(body_id, i, **self.props['dynamics'][body_type])

    def create_ball(self, urdf_name, init_pose=None, init_speed=None,
                    noise=None, init_vel=None):
        """
        @param urdf_name: The id for the specific pybullet object.
        @param init_pose: The initial position of the ball (override randomization).
        @param init_speed: The max speed of the ball (override known speed parameter).
        @param noise: The noise and if it should be present in the location of the object.
        @param init_vel: The initial velocity of the ball (override randomization).
        @return: The ball id if the creation was successful.
        """
        if urdf_name in self.urdf_paths:
            zero_orient = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
            if init_pose:
                ball_pos = init_pose["pos"]
                self.init_ball_pos = ball_pos
            else:
                ball_pos = [
                    random.uniform(
                        self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(
                        self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(
                        self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
                ]
                self.init_ball_pos = None
            self._ball_id = p.loadURDF(
                self.urdf_paths[urdf_name], ball_pos, zero_orient)
            self.configure_dynamics(self._ball_id, "ball")

            # initialize the ball with some speed
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
        """
        Creates instance based car properties(pose,vel,orient) and configures car dynamics.
        @param urdf_name: The id for the specific pybullet object.
        @param init_pose: The initial position of the ball (override randomization).
        @param noise: The noise and if it should be present in the location of the object.
        @param car_props: Configuration based car properties.
        @return: The car id if the creation was successful.
        """

        # set the spawn location for the car
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
                    random.uniform(
                        self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(
                        self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(
                        self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
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

            # configures dynamics of the car
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
        """
        Removes a car from being tracked in the _cars and _car_data lists.
        @param car_id: The id of the car in the simulator class.
        @return: Whether the deletion was successful.
        """
        if car_id not in self._cars:
            return False

        p.removeBody(car_id)
        del self._cars[car_id]
        del self._car_data[car_id]
        return True

    def set_car_command(car_id, msg):
        """
        Set the command for the car
        """
        cars[car_id].setCmd(msg)

    def step(self, car_cmd, dt):
        """
        Moves the sim forward one timestep, checking if a goal is score to end the sim round.
        @param dt: The change in time (delta-t) for this sim step.
        """
        # PyBullet steps at 240hz
        p_dt = 1.0 / 240.0
        if self._ball_id is not None:
            ball_contacts = p.getContactPoints(bodyA=self._ball_id)
            # TODO: need to decrease ball velocity
            linear, angular = self.get_ball_velocity()
            new_linear,new_angular = self.get_decreased_velocity(linear,angular,p_dt)
            p.resetBaseVelocity(self._ball_id,new_linear,new_angular)
            
            for contact in ball_contacts:
                if contact[2] in self._cars:
                    self.touchedLast = contact[2]
                elif contact[2] == self._goal_a_id:
                    self.scored = True
                    self.winner = "A"
                elif contact[2] == self._goal_b_id:
                    self.scored = True
                    self.winner = "B"
            

        
        for _ in range(round(dt / p_dt)):
            # step kinematic objects independently, at max possible rate
            for car in self._cars.values():
                car.step(p_dt)
            p.stepSimulation()
    def get_decreased_velocity(self,linear,angular,dt):
        x_vel,y_vel,z_vel=linear[0],linear[1],linear[2]
        
        current_speed = math.sqrt(x_vel*x_vel +y_vel*y_vel)
        # r*cos(theta), r*sin(theta)
        
        
        if current_speed < 0.001: # a very small number
            return (0,0,z_vel),angular
        # for each time step, new_vel = old_vel - myu_r*dt
        angle = math.atan(y_vel/x_vel)
        current_speed = current_speed - 0.17 * dt
        
        new_x_vel = current_speed*math.cos(angle) # v*cos(theta)
        new_y_vel = current_speed*math.sin(angle) # v*sin(theta)
        
        
        return (new_x_vel,new_y_vel,z_vel),angular
        
    def get_car_pose(self, id, add_noise=False):

        if id not in self._cars:
            return None

        noise = self._car_data[id]['noise']
        if add_noise:
            return self._cars[id].get_pose(noise=noise)
        else:
            return self._cars[id].get_pose(noise=None)

    def get_car_velocity(self, id):
        """Returns a tuple of linear and angular velocity for the car."""
        if id not in self._cars:
            return None

        return self._cars[id].get_velocity()

    def set_car_command(self, id, cmd):
        if id not in self._cars:
            return None

        return self._cars[id].setCmd(cmd)

    def get_ball_pose(self, add_noise=False):
        """@param add_noise: State whether you want noise to get the ball position (default=False)."""
        if self._ball_id is None:
            return None
        pos, _ = p.getBasePositionAndOrientation(self._ball_id)

        if add_noise and self.ball_noise:
            if np.random.uniform() < self.ball_noise['dropout']:
                return None, None
            else:
                pos = np.random.normal(pos, self.ball_noise['pos'])

        return pos, p.getQuaternionFromEuler([0, 0, 0])

    def get_ball_velocity(self):
        if self._ball_id is None:
            return None

        return p.getBaseVelocity(self._ball_id)

    def reset(self, spawn_bounds, car_properties, ball_init_pose, ball_init_speed):
        """
        Resets the ball, score, winner, spawn bounds, cars and ball.
        @param spawn_bounds: The new spawn bounds.
        @param car_properties: The new car properties.
        """
        self.scored = False
        self.winner = None
        self.touched_last = None
        if ball_init_pose is not None:
            self.init_ball_pos = ball_init_pose
        if ball_init_speed is not None:
            self._speed_bound = ball_init_speed

        self.spawn_bounds = spawn_bounds
        self.reset_ball()
        for car in self._cars.values():
            self.reset_car(car, car_properties)

    def reset_car(self, car, car_properties):
        """
        Loops over the cars and generates new initial positions (if they were not specified).
        @param car_properties: The new car config properties.
        """
        # reset the car properties in advance
        car.set_properties(car_properties)
        car_pos = self._car_data[car.id]["init_pos"]
        car_orient = self._car_data[car.id]["init_orient"]

        if car_pos is None:
            car_pos = self.generate_new_car_pos()

        while self.check_if_pos_overlap(car_pos):
            car_pos = self.generate_new_car_pos()

        if car_orient is None:
            car_orient = [0, 0, random.uniform(0, 2 * math.pi)]
        car.reset(car_pos, car_orient)

    def check_if_pos_overlap(self, car_pos):
        """
        Checks if two cars spawn bounds overlap with each other.
        @param car_pos: The position of the car.
        @return: Whether overlap happens (true = need to generate new bounds).
        """
        for car in self._cars.values():
            overlap = car.check_overlap(car_pos)
            if overlap:
                return True

        return False

    def generate_new_car_pos(self):
        car_pos = [random.uniform(self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                   random.uniform(
            self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
            random.uniform(self.spawn_bounds[2][0], self.spawn_bounds[2][1])]
        return car_pos

    def reset_ball(self):

        if self._ball_id is not None:
            ball_pos = self.init_ball_pos
            if ball_pos is None:
                ball_pos = [
                    random.uniform(
                        self.spawn_bounds[0][0], self.spawn_bounds[0][1]),
                    random.uniform(
                        self.spawn_bounds[1][0], self.spawn_bounds[1][1]),
                    random.uniform(
                        self.spawn_bounds[2][0], self.spawn_bounds[2][1]),
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
