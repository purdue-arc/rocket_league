#!/usr/bin/env python

"""Contains pure pursuit helper functions.

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
import numpy as np
import math
from tf.transformations import euler_from_quaternion

def find_intersection(path_seg, bot_path, lookahead_dist):
    """
    Uses quadratic formula to find intersections between
    the lookahead radius and the path.
    """

    a = np.dot(path_seg, path_seg)
    b = 2 * np.dot(path_seg, bot_path)
    c = np.dot(bot_path, bot_path) - (lookahead_dist * lookahead_dist)
    discrim = (b*b) - (4*a*c)

    if discrim >= 0:
        discrim = math.sqrt(discrim)
        t1 = (-b - discrim)/(2*a)
        t2 = (-b + discrim)/(2*a)

        if t1 >= 0 and t1 <= 1:
            return path_seg * t1
        if t2 >= 0 and t2 <= 1:
            return path_seg * t2
    return None

def calculate_dist(pos, bot_pos, bot_orient, lookahead_dist):
    """Determines arc distance to reach intersection point."""
    _, _, bot_yaw = euler_from_quaternion(bot_orient)
    a = -math.tan(bot_yaw)
    c = math.tan(bot_yaw) * bot_pos[0] - bot_pos[1]
    dist = math.sqrt(math.pow(a,2) + 1)
    x = abs(((a * pos[0]) + pos[1] + c) / dist)
    radius = (lookahead_dist * lookahead_dist)/(2 * x)

    dist = abs(np.linalg.norm(pos - bot_pos))
    arc_angle = 2 * math.asin((dist / 2) / radius)
    return arc_angle * radius

def calculate_angle(intersect_pos, bot_pos, bot_orient, lookahead_dist):
    """
    Determines angle from heading to the intersection point.
    Angle is from -pi to pi, with 0 at car's heading.
    """
    _, _, bot_yaw = euler_from_quaternion(bot_orient)

    bot_line_x = bot_pos[0] + math.cos(bot_yaw) * lookahead_dist
    bot_line_y = bot_pos[1] + math.sin(bot_yaw) * lookahead_dist
    bot_line = np.array([bot_line_x, bot_line_y, 0])

    tang_line = intersect_pos - bot_line
    dist = np.linalg.norm(tang_line)
    angle = (2 * math.asin((dist / 2)/lookahead_dist))
    sign = np.sign(np.cross(tang_line, bot_line))[2]
    return angle * sign

def get_angular_speed(target_vel, angle, dist):
    """Relates path to the angular velocity."""
    dt = dist / target_vel
    return angle / dt
