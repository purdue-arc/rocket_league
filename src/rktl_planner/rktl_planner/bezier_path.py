"""
A module for handling Bezier curves and paths.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import math
from rktl_planner import BezierCurve
from rktl_msgs.msg import BezierPath as BezierPathMsg
from rclpy import Duration
from geometry_msgs.msg import Vector3
from std_msgs.msg import Duration as DurationMsg


class BezierPath:
    """
    A class representing a Bezier curve path, which is a `BezierCurve` and a duration. If `msg`,
    `bezier_curve`, and/or `duration` is given as kwargs, they are used to convert from
    `rktl_msgs/BezierPathMsg`, used as the bezier curve for this path, and/or used for as the
    duration path; respectively. Otherwise, args is used to initalize this object. If one argument
    is given, it is expected to be of type `rktl_msgs.msgs.BezierPathMsg`. If two arguments are
    given, the first should be either a `BezierCurve` object or a list of control points to generate
    a `BezierCurve` object. The second argument should be a `std_msgs.msg.Duration` or a `float`
    dictating how long this segment should last.    
    """

    def __init__(self, *args, **kwargs):
        self.bezier_curve = None
        self.duration = None

        if args:
            if len(args) == 1 and type(args[0]) is BezierPathMsg:
                self.bezier_curve = BezierCurve(
                    args[0].order, args[0].control_points)
                self.duration = args[0].duration.data
            elif len(args) == 2:
                if type(args[0]) is BezierCurve:
                    self.bezier_curve = args[0]
                elif type(args[0]) is list:
                    self.bezier_curve = BezierCurve(args[0])
                else:
                    raise ValueError(f'Unknown argument {args[0]!r}')
                if type(args[1]) is Duration:
                    self.duration = args[1]
                elif type(args[1]) is float:
                    self.duration = Duration(args[1])
                else:
                    raise ValueError(f'Unknown argument {args[1]!r}')
            else:
                raise ValueError(f'Unknown arguments {args!r}')
        if kwargs:
            for k, v in kwargs.items():
                if k == 'msg':
                    if type(v) is not BezierPathMsg:
                        raise ValueError(
                            f'{k!r} must be {BezierPathMsg}, got {type(v)}')
                    self.bezier_curve = v.bezier_curve
                    self.duration = v.duration
                elif k == 'bezier_curve':
                    if type(v) is not BezierCurve:
                        raise ValueError(
                            f'{k!r} must be {BezierCurve}, got {type(v)}')
                    self.bezier_curve = v
                elif k == 'duration':
                    if type(v) is not Duration:
                        raise ValueError(
                            f'{k!r} must be {Duration}, got {type(v)}')
                    self.duration = v
                else:
                    raise ValueError(f'Unknown keyword argument {k!r}')

    def __repr__(self):
        """Returns a string representation of the instance."""
        return f'{self.__class__.__name__}({self.bezier_curve!r}, [{self.duration!r}])'

    def __str__(self):
        """Returns a string representation of the instance."""
        return f'{self.__class__.__name__}[{self.bezier_curve!s}, {self.duration!s}ns]'

    def to_param(self, secs):
        """Returns the parameter value corresponding to a time in seconds."""
        return (secs.to_sec() if type(secs) is Duration else float(secs)) / self.duration.to_sec()

    def from_param(self, vec):
        """Returns a `Vector3` object corresponding to a parameter value."""
        dt = 1.0 / self.duration.to_sec()
        return Vector3(vec.x * dt, vec.y * dt, vec.z * dt)

    def at(self, secs):
        """Returns a `Vector3` object representing the position on the path at a given time in seconds."""
        t = self.to_param(secs)
        return self.bezier_curve.at(t)

    def vel_at(self, secs):
        """Returns a `Vector3` object representing the velocity on the path at a given time in seconds."""
        t = self.to_param(secs)
        vec = self.bezier_curve.deriv(t)
        return self.from_param(vec)

    def speed_at(self, secs):
        """Returns the (tangential) speed on the path at a given time in seconds."""
        vel = self.vel_at(secs)
        return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

    def accel_at(self, secs):
        """Returns a Vector3 object representing the acceleration on the path at a given time in seconds."""
        t = self.to_param(secs)
        dv = self.bezier_curve.hodograph().hodograph().at(t)
        dt = self.duration.to_sec()
        return Vector3(dv.x/(dt**2), dv.y/(dt**2), dv.z/(dt**2))

    def angle_at(self, secs):
        """Returns the angle of the tangent vector of the curve at a given time in seconds."""
        vel = self.vel_at(secs)
        if vel.x == 0 and vel.y == 0:
            vel = self.accel_at(secs)
        return math.atan2(vel.y, vel.x)

    def angular_vel_at(self, secs):
        """Returns the angular velocity of the curve at a given time in seconds."""
        vel = self.vel_at(secs)
        accel = self.accel_at(secs)
        return (vel.x * accel.x - vel.y * accel.y) / (vel.x ** 2 + vel.y ** 2)

    def to_msg(self):
        """Returns a `rktl_msg/BezierPathMsg` object representing the `BezierPath` object."""
        duration_msg = DurationMsg(self.duration)
        msg = BezierPathMsg(
            order=self.bezier_curve.order,
            control_points=self.bezier_curve.control_points,
            duration=duration_msg
        )
        return msg

    def split(self, secs):
        """Splits this path into 2 paths at a given time in seconds. Returns the two new `BezierPath` objects."""
        t = self.to_param(secs)
        curve1, curve2 = self.bezier_curve.de_casteljau(t)
        duration1 = Duration(secs)
        duration2 = Duration(self.duration.to_sec() - secs)
        path1 = BezierPath(bezier_curve=curve1, duration=duration1)
        path2 = BezierPath(bezier_curve=curve2, duration=duration2)
        return path1, path2
