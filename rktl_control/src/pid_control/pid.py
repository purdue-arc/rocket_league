"""A very basic PID controller.
License:
  BSD 3-Clause License
  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

class PIDController(object):
    """A very basic PID controller."""

    def __init__(self, kp, ki, kd, anti_windup=0.1, deadband=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.anti_windup = anti_windup
        self.deadband = deadband

        self.reset()

    def step(self, error, delta_t):
        """One time step."""
        if abs(error) < self.deadband:
            return self._last_output

        kk = self.kp * error
        if abs(error) < self.anti_windup:
            self._integral += error * delta_t
        ii = self.ki * self._integral
        dd = self.kd * (error - self._last_error) / delta_t

        output = kk + ii + dd
        self._last_output = output
        return output

    def reset(self):
        """Reset to initialse state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_output = 0.0
