"""Contains ROS message conversions.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import numpy as np


def odom_to_array(msg):
    """Convert odom msg to numpy arrays."""

    p, q, pc = pose_covar_to_array(msg.pose)
    l, q, tc = twist_covar_to_array(msg.twist)

    return p, q, pc, l, q, tc


def pose_covar_to_array(msg):
    """Convert pose with covariance msg to numpy arrays."""

    p, q = pose_to_array(msg.pose)
    c = np.array(msg.covariance)
    c = np.reshape(c, (6, 6))

    return p, q, c


def twist_covar_to_array(msg):
    """Convert twist with covariance msg to numpy arrays."""

    l, a = twist_to_array(msg.twist)
    c = np.array(msg.covariance)
    c = np.reshape(c, (6, 6))

    return l, a, c


def pose_to_array(msg):
    """Convert pose msg to numpy arrays."""

    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y,
                  msg.orientation.z, msg.orientation.w])

    return p, q


def twist_to_array(msg):
    """Convert twist msg to numpy arrays."""

    l = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    a = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    return l, a
