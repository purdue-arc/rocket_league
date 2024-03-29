"""
This module implements a Bezier curve class with various methods. A Bezier
curve is defined by its order and control points.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

from geometry_msgs.msg import Point
from math import pow


class BezierCurve:
    """
    A class representing a Bezier Curve and a duration. If `order` and `control_points` are given in
    kwargs, then it is used to initialize the instance. Otherwise, args is used to initialize the 
    instance. When not using kwargs, 4 options exist. Either a list of control points
    (geometry_msgs.msg.Points), the order of the bezier curve, both the order and the list of
    control points, or the control points as indiviudal arguments can be given.

    For example, All examples below give an instance of a `BezierCurve` of order 3. When points
    are specified, they are the control points used for the curve.

    ```python
    curve = BezierCurve(3)
    curve = BezierCurve(3, [Point(0, 0, 0), Point(1, 2, 3), Point(3, 2, 1), Point(4, 2, 0)])
    curve = BezierCurve([Point(0, 0, 0), Point(1, 2, 3), Point(3, 2, 1), Point(4, 2, 0)]))
    curve = BezierCurve(Point(0, 0, 0), Point(1, 2, 3), Point(3, 2, 1), Point(4, 2, 0)))
    curve = BezierCurve(order=3, control_points=[Point(0, 0, 0), Point(1, 2, 3), Point(3, 2, 1), Point(4, 2, 0)]))
    curve = BezierCurve(control_points=[Point(0, 0, 0), Point(1, 2, 3), Point(3, 2, 1), Point(4, 2, 0)]))```
    """

    _coefficients = [[1]]
    """Table used internally to generate the coefficients of the bezier curve faster"""

    def __init__(self, *args, **kwargs):
        self.control_points = None
        self.order = None

        if args:
            if len(args) == 1 and type(args[0]) is list:
                self.control_points = args[0]
            elif len(args) == 1 and type(args[0]) is int:
                self.order = args[0]
            elif len(args) == 2 and type(args[0]) is int and type(args[1]) is list:
                self.order = args[0]
                self.control_points = args[1]
            else:
                self.control_points = args
        if kwargs:
            for k, v in kwargs.items():
                if k == 'order':
                    if type(v) is not int:
                        raise ValueError(f'{k!r} must be {int}, got {type(v)}')
                    self.order = v
                elif k == 'control_points':
                    if type(v) is not list:
                        raise ValueError(
                            f'{k!r} must be {list}, got {type(v)}')
                    self.control_points = v
                else:
                    raise ValueError(f'Unknown keyword argument {k!r}')

        if self.order is not None and self.control_points is None:
            self.control_points = [Point()] * (self.order + 1)
        elif self.control_points is not None and self.order is None:
            self.order = len(self.control_points) - 1
        elif self.order is None and self.control_points is None:
            raise ValueError(
                'Neither \'order\' nor \'control_points\' was specified')
        elif self.order and self.control_points and self.order != len(self.control_points) - 1:
            raise ValueError(
                'Value of \'order\' and length of \'control_points\' contridict each other')

        for i, p in enumerate(self.control_points):
            if type(p) is not Point:
                raise ValueError(
                    f'Element {i} of \'control_points\' must be {Point}, got {type(p)}')

        self.my_hodograph = self if self.order == 0 else None
        self.coefficients = BezierCurve.calc_coefficients(self.order + 1)

    def __repr__(self):
        """Returns a string representation of the instance."""
        def point_str(p): return f'({p.x:.2f}, {p.y:.2f}, {p.z:.2f})'
        strs = [point_str(x) for x in self.control_points[:2]]
        if len(self.control_points) > 5:
            strs += ['...'] + [point_str(x) for x in self.control_points[-2:]]
        else:
            strs += [point_str(x) for x in self.control_points[2:]]
        points = ', '.join(strs)
        return f'{self.__class__.__name__}({self.order}, [{points}])'

    def __str__(self):
        """Returns a string representation of the instance."""
        return f'{self.__class__.__name__}: order {self.order}'

    def calc_coefficients(n):
        """Calculates and returns the list of binomial coefficients for a given n."""
        if len(BezierCurve._coefficients) >= n:
            return BezierCurve._coefficients[n - 1]
        prevRow = [0] + BezierCurve.calc_coefficients(n - 1) + [0]
        row = [prevRow[i] + prevRow[i + 1] for i in range(n)]
        BezierCurve._coefficients += [row]
        return row

    def at(self, t):
        """Returns a Point object representing the location of the curve at the given parameter value `t`."""
        point = Point()
        basis = [c * pow(t, i) * pow(1 - t, self.order - i)
                 for i, c in enumerate(self.coefficients)]
        for b, p in zip(basis, self.control_points):
            point.x += b * p.x
            point.y += b * p.y
            point.z += b * p.z
        return point

    def hodograph(self):
        """Returns a `BezierCurve` object representing the hodograph (derivative) of the curve."""
        if not self.my_hodograph:
            control_points = []
            for i in range(self.order):
                p = Point()
                p1 = self.control_points[i + 1]
                p2 = self.control_points[i]
                p.x = self.order * (p1.x - p2.x)
                p.y = self.order * (p1.y - p2.y)
                p.z = self.order * (p1.z - p2.z)
                control_points.append(p)
            self.my_hodograph = BezierCurve(
                order=self.order - 1, control_points=control_points)
        return self.my_hodograph

    def deriv(self, t):
        """Returns a `geometry_msgs/Point` object representing the derivative of the curve at the given parameter value `t`."""
        return self.hodograph().at(t)

    def de_casteljau(self, t):
        """Splits this curve into 2 curves at the the point specified by `t`. Returns the two new `BezierCurve` objects."""
        # b[j][i] = b_i^(j)
        t0 = float(t)
        t1 = 1 - t0
        b = [self.control_points]
        for j in range(self.order):
            b.append([])
            for i in range(self.order - j):
                b[j + 1].append(Point())
                b[j + 1][i].x = b[j][i].x * t1 + b[j][i + 1].x * t0
                b[j + 1][i].y = b[j][i].y * t1 + b[j][i + 1].y * t0
        points1 = [b[i][0] for i in range(self.order + 1)]
        points2 = [b[self.order - i][i] for i in range(self.order + 1)]
        return BezierCurve(points1), BezierCurve(points2)
