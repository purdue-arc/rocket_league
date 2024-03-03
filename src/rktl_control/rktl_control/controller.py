#!/usr/bin/env python3
"""
Customizable controller for the car. It implements either a PID controller or a
lead-lag controller.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rclpy
import sys
from nav_msgs.msg import Odometry
from rktl_msgs.msg import ControlCommand, ControlEffort
from math import atan, cos, sin


class PIDController(object):
    """A very basic PID controller."""

    def __init__(self, kp, ki, kd, anti_windup, deadband, delta_t):
        # Constants
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.ANTI_WINDUP = anti_windup
        self.DEADBAND = deadband
        self.DELTA_T = delta_t

        # State variables
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0

    def step(self, error):
        """One time step."""
        if abs(error) < self.DEADBAND:
            return self.last_output

        kk = self.KP * error
        if abs(error) < self.ANTI_WINDUP:
            self.integral += error * self.DELTA_T
        ii = self.KI * self.integral
        dd = self.KD * (error - self.last_error) / self.DELTA_T

        output = kk + ii + dd
        self.last_output = output
        return output

    def reset(self):
        """Reset to initial state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_output = 0.0


class LeadLagController(object):
    """A simple discrete lead-lag controller."""

    def __init__(self, k_lead, a_lead, b_lead, a_lag, b_lag):
        # Constants
        self.GAIN_LEAD = k_lead
        self.ALPHA_LEAD = a_lead
        self.BETA_LEAD = b_lead
        self.ALPHA_LAG = a_lag
        self.BETA_LAG = b_lag

        # State variables
        self.prev_R_lead = 0.0
        self.prev_R_lag = 0.0

    def step(self, error):
        """One time step."""
        R_lead = error - self.BETA_LEAD * self.prev_R_lead
        U_lead = R_lead + self.ALPHA_LEAD * self.prev_R_lead
        R_lag = U_lead - self.BETA_LAG * self.prev_R_lag
        U_lag = R_lag + self.ALPHA_LAG * self.prev_R_lag
        effort = self.GAIN_LEAD * U_lag

        self.prev_R_lead = R_lead
        self.prev_R_lag = R_lag

        return effort

    def reset(self):
        """Reset to initial state."""
        self.prev_R_lead = 0.0
        self.prev_R_lag = 0.0


class Controller(object):
    """Controller for car."""

    def __init__(self):
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('controller')

        # Constants
        self.MAX_SPEED = self.node.declare_parameter('/cars/throttle/max_speed', 2.3).value
        self.STEERING_THROW = self.node.declare_parameter('/cars/steering/max_throw', 0.1826).value
        self.BODY_LENGTH = self.node.declare_parameter('/cars/length', 0.12).value

        self.MIN_THROTTLE_EFFORT = self.node.declare_parameter('~limits/throttle/min', -1.0).value
        self.MAX_THROTTLE_EFFORT = self.node.declare_parameter('~limits/throttle/max',  1.0).value
        self.MIN_STEERING_EFFORT = self.node.declare_parameter('~limits/steering/min', -1.0).value
        self.MAX_STEERING_EFFORT = self.node.declare_parameter('~limits/steering/max',  1.0).value

        self.PUBLISH_EARLY = self.node.declare_parameter('~open_loop/publish_early', True).value

        self.CONTROLLER_TYPE = self.node.declare_parameter('~controller_type', 'none').value

        # Make closed loop velocity controller
        if self.CONTROLLER_TYPE == 'lead_lag':
            self.controller = LeadLagController(
                self.node.declare_parameter('~lead/gain').value,
                self.node.declare_parameter('~lead/alpha').value,
                self.node.declare_parameter('~lead/beta').value,
                self.node.declare_parameter('~lag/alpha').value,
                self.node.declare_parameter('~lag/beta').value)
        elif self.CONTROLLER_TYPE == 'pid':
            self.controller = PIDController(
                self.node.declare_parameter('~pid/kp').value,
                self.node.declare_parameter('~pid/ki').value,
                self.node.declare_parameter('~pid/kd').value,
                self.node.declare_parameter('~pid/anti_windup').value,
                self.node.declare_parameter('~pid/deadband').value,
                1.0 / self.node.declare_parameter('~rate', 10.0).value)
        elif self.CONTROLLER_TYPE == 'none':
            self.controller = None
        else:
            raise NotImplementedError(f"unrecognized controller type: {self.node.declare_parameter('controller_type').value}")

        # State variables
        self.vr_ref = None
        self.psi_ref = None

        # Publishers
        self.pub = self.node.create_publisher(ControlEffort, 'effort', 1)

        # Subscribers
        self.node.create_subscription(ControlCommand, 'command', self.command_cb, 1)
        self.node.create_subscription(Odometry, 'odom', self.odom_cb, 1)

        # trust that odom_cb runs at proper rate
        rclpy.spin(self.node)

    def command_cb(self, cmd_msg):
        """Callback for command messages for car."""
        # calculate reference steering angle and rear wheel velocities
        cos_beta = cos(sin(cmd_msg.curvature * self.BODY_LENGTH / 2.0))
        self.vr_ref = cmd_msg.velocity * cos_beta
        self.psi_ref = atan(self.BODY_LENGTH * cmd_msg.curvature / cos_beta)

        # get a jump on latency if open loop
        if self.controller is None and self.PUBLISH_EARLY:
            self.odom_cb(None)

    def odom_cb(self, odom_msg):
        """Callback for odom messages from car."""
        if self.vr_ref is None or self.psi_ref is None:
            return

        if self.controller is not None:
            # calculate throttle effort (closed loop)
            error = self.vr_ref - odom_msg.twist.twist.linear.x
            throttle_effort = self.controller.step(error)
        else:
            # calculate throttle effort (open loop)
            throttle_effort = self.vr_ref / self.MAX_SPEED

        # calculate steering effort (open loop)
        steering_effort = self.psi_ref / self.STEERING_THROW

        # enforce actuator saturation limits
        throttle_effort = max(
            min(throttle_effort, self.MAX_THROTTLE_EFFORT),
            self.MIN_THROTTLE_EFFORT)
        steering_effort = max(
            min(steering_effort, self.MAX_STEERING_EFFORT),
            self.MIN_STEERING_EFFORT)

        # publish actuator efforts
        msg = ControlEffort()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.throttle = throttle_effort
        msg.steering = steering_effort
        self.pub.publish(msg)

def main():
    Controller()

if __name__ == "__main__":
    main()