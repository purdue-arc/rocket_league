#!/usr/bin/env python

"""Contains ROS message conversions.

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
    c = np.reshape(c, (6,6))

    return p, q, c

def twist_covar_to_array(msg):
    """Convert twist with covariance msg to numpy arrays."""

    l, a = twist_to_array(msg.twist)
    c = np.array(msg.covariance)
    c = np.reshape(c, (6,6))

    return l, a, c

def pose_to_array(msg):
    """Convert pose msg to numpy arrays."""

    p = np.array([msg.position.x, msg.position.y, msg.position.z])
    q = np.array([msg.orientation.x, msg.orientation.y, \
                    msg.orientation.z, msg.orientation.w])

    return p, q

def twist_to_array(msg):
    """Convert twist msg to numpy arrays."""

    l = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    a = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    return l, a