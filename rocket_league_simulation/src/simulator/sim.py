#!/usr/bin/env python3

"""Contains the Sim class.

License:
  BSD 3-Clause License
  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# 3rd party modules
import math
import os
import pybullet as p
import pybullet_data as p_data
import random

# car_urdf_path = "/home/zer0/catkin_ws/src/rocket_league/rocket_league_simulation/src/simulator/ur_temp/car.urdf"

class Sim(object):
    """Oversees components of the simulator"""

    def __init__(self, ball_urdf_path, car_urdf_path, render_enabled):
        if render_enabled:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(p_data.getDataPath())
        self.planeID = p.loadURDF("plane.urdf")

        startPos = [0, 0, 1]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.ballID = p.loadURDF(ball_urdf_path, startPos, startOrientation)
        
        
        self.carID = p.loadURDF(car_urdf_path, [0,0,0], p.getQuaternionFromEuler([0,0,0]))
        self.steering_angle = 0
        self.length = 0.5
        
        self.car_handle = p.createConstraint(self.carID, -1,-1,-1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,1])

        p.resetBasePositionAndOrientation(self.carID, [0,0,0.1], p.getQuaternionFromEuler([0,0,0]))
        p.setGravity(0, 0, -10)
        self.running = True

    def step(self, throttle, steering, dt):
        """Advance one time-step in the sim."""
        if self.running:
            # implementing bicycle model
            self.steering_angle += steering*dt
            position, orientation = p.getBasePositionAndOrientation(self.carID)
        # global_angle = math.atan(position[1]/position[0])
            heading = p.getEulerFromQuaternion(orientation)[2]
            # input_array = np.array([throttle, steering, 1])
            # transform = np.array(
            #     [ 
                    
            #     ]
            # )
            x_vel = throttle * math.cos(heading + steering/2)
            y_vel = throttle * math.sin(heading + steering/2)
            w = (throttle * math.tan(heading + steering) * math.cos(heading + steering/2))/self.length
            
            position = (position[0] + x_vel*dt, position[1] + y_vel*dt, position[2])
            orientation = p.getQuaternionFromEuler([0, 0, heading + w * dt])
            
            p.changeConstraint(self.car_handle, position, orientation)
            
            p.stepSimulation()

    def reset(self):
        """Reset simulator to original state."""
        pass
