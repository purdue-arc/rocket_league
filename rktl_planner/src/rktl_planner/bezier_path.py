#!/usr/bin/env python3

from rktl_planner import BezierCurve
from rktl_planner.msg import BezierPath as BezierPathMsg
from rospy import Duration
from geometry_msgs.msg import Vector3
from std_msgs.msg import Duration as DurationMsg
from math import sqrt


class BezierPath:
    def __init__(self, *args, **kwargs):
        self.bezier_curve = None
        self.duration = None

        if args:
            if len(args) == 1 and type(args[0]) is BezierPathMsg:
                self.bezier_curve = BezierCurve(args[0].bezier_curve)
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
                    self.duration = Duration(args[1 ])
                else:
                    raise ValueError(f'Unknown argument {args[1]!r}')
            else:
                raise ValueError(f'Unknown arguments {args!r}')
        if kwargs:
            for k, v in kwargs.items():
                if k == 'msg':
                    if type(v) is not BezierPathMsg:
                        raise ValueError(f'{k!r} must be {BezierPathMsg}, got {type(v)}')
                    self.bezier_curve = v.bezier_curve
                    self.duration = v.duration
                elif k == 'bezier_curve':
                    if type(v) is not BezierCurve:
                        raise ValueError(f'{k!r} must be {BezierCurve}, got {type(v)}')
                    self.bezier_curve = v
                elif k == 'duration':
                    if type(v) is not Duration:
                        raise ValueError(f'{k!r} must be {Duration}, got {type(v)}')
                    self.duration = v
                else:
                    raise ValueError(f'Unknown keyword argument {k!r}')

    def __repr__(self):
        return f'{self.__class__.__name__}({self.bezier_curve!r}, [{self.duration!r}])'

    def __str__(self):
        return f'{self.__class__.__name__}[{self.bezier_curve!s}, {self.duration!s}ns]'

    def to_param(self, secs):
        return (secs.to_sec() if type(secs) is Duration else float(secs)) / self.duration.to_sec()

    def from_param(self, vec):
        dt = 1.0 / self.duration.to_sec()
        return Vector3(vec.x * dt, vec.y * dt, vec.z * dt)

    def at(self, secs):
        t = self.to_param(secs)
        return self.bezier_curve.at(t)
    
    def vel_at(self, secs):
        t = self.to_param(secs)
        vec = self.bezier_curve.deriv(t)
        return self.from_param(vec)
    
    def speed_at(self, secs):
        vel = self.vel_at(secs)
        return sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)

    def accel_at(self, secs):
        t = self.to_param(secs)
        dx = self.bezier_curve.hodograph().at(t)
        dv = self.bezier_curve.hodograph().hodograph().at(t)
        dt = self.duration.to_sec()
        denominator = sqrt(dx.x * dx.x + dx.y * dx.y + dx.z * dx.z) * dt * dt
        if denominator == 0.0:
            return 0.0
        numerator = dx.x * dv.x + dx.y * dv.y + dx.z * dv.z
        return numerator / denominator

    def heading_at(self, secs):
        vel = self.vel_at(secs)
        speed = sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
        if speed == 0.0:
            return Vector3()
        return Vector3(vel.x / speed, vel.y / speed, vel.z / speed)

    def to_msg(self):
        bezier_curve_msg = self.bezier_curve.to_msg()
        duration_msg = DurationMsg(self.duration)
        msg = BezierPathMsg(bezier_curve=bezier_curve_msg, duration=duration_msg)
        return msg

    def split(self, secs):
        t = self.to_param(secs)
        curve1, curve2 = self.bezier_curve.de_casteljau(t)
        duration1 = Duration(secs)
        duration2 = Duration(self.duration.to_sec() - secs)
        path1 = BezierPath(bezier_curve=curve1, duration=duration1)
        path2 = BezierPath(bezier_curve=curve2, duration=duration2)
        return path1, path2
