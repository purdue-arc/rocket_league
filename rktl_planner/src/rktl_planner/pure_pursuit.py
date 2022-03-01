"""Contains pure pursuit helper functions.
License:
  BSD 3-Clause License
  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
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


def calculate_lat_error(intersect_pos, bot_pos, bot_orient, lookahead_dist):
    """Determines lateral error from intersection point."""

    _, _, bot_yaw = euler_from_quaternion(bot_orient)
    a = -math.tan(bot_yaw)
    c = (math.tan(bot_yaw) * bot_pos[0]) - bot_pos[1]
    dist = math.sqrt(math.pow(a, 2) + 1)
    x = abs((a * intersect_pos[0]) + intersect_pos[1] + c) / dist

    bot_line_x = bot_pos[0] + (math.cos(bot_yaw) * lookahead_dist)
    bot_line_y = bot_pos[1] + (math.sin(bot_yaw) * lookahead_dist)
    bot_line = np.array([bot_line_x, bot_line_y, 0])
    tang_line = intersect_pos - bot_line
    sign = np.sign(np.cross(tang_line, bot_line))[2]
    return x * sign


def calculate_turn_rad(intersect_pos, bot_pos, bot_orient, lookahead_dist, bkw):
    """Determines turning radius to reach intersection point."""

    _, _, bot_yaw = euler_from_quaternion(bot_orient)
    a = -math.tan(bot_yaw)
    c = (math.tan(bot_yaw) * bot_pos[0]) - bot_pos[1]
    dist = math.sqrt(math.pow(a, 2) + 1)
    x = abs((a * intersect_pos[0]) + intersect_pos[1] + c) / dist
    radius = (lookahead_dist * lookahead_dist)/(2 * x)

    bot_line_x = math.cos(bot_yaw) * lookahead_dist
    bot_line_y = math.sin(bot_yaw) * lookahead_dist
    bot_line = np.array([bot_line_x, bot_line_y, 0])
    sign = np.sign(np.cross(intersect_pos - bot_pos, bot_line))[2]
    if bkw:
        sign *= -1

    return radius * sign


def calculate_angle(intersect_pos, bot_pos, bot_orient, lookahead_dist, bkw):
    """
    Determines angle from heading to the intersection point.
    Angle is from -pi to pi, with 0 at car's heading.
    """
    _, _, bot_yaw = euler_from_quaternion(bot_orient)

    bot_line_x = math.cos(bot_yaw) * lookahead_dist
    bot_line_y = math.sin(bot_yaw) * lookahead_dist
    bot_line = np.array([bot_line_x, bot_line_y, 0])

    tang_line = intersect_pos - (bot_pos + bot_line)
    dist = np.linalg.norm(tang_line)
    angle = (2 * math.asin(round((dist / 2) / lookahead_dist, 6)))
    sign = np.sign(np.cross(intersect_pos - bot_line, bot_line))[2]

    # if angle < 0:
    #     angle += math.pi
    # else:
    #     angle -= math.pi
    # if bkw:
        # angle = math.pi - angle
    return angle * sign


def get_angular_speed(linear_vel, turn_rad):
    """Relates path to the angular velocity."""
    if turn_rad != 0:
        return linear_vel / turn_rad
    else:
        return 0
