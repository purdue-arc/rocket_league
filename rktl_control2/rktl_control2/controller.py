"""
Customizable controller for the car. It implements either a PID controller or a
lead-lag controller.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rktl_msgs.msg import ControlCommand, ControlEffort
from math import atan, cos, sin

# Define our own Controller Node
class ControllerNode(Node):

    # Constructor
    def __init__(self):
        super().__init__("controller") # Name of Node in ROS 2 Communications
        
        self.MAX_SPEED = self.get_parameter('/cars/throttle/max_speed')
        self.STEERING_THROW = self.get_parameter('/cars/steering/max_throw')
        self.BODY_LENGTH = self.get_parameter('/cars/length')

        self.MIN_THROTTLE_EFFORT = self.get_parameter('~limits/throttle/min', -1.0)
        self.MAX_THROTTLE_EFFORT = self.get_parameter('~limits/throttle/max',  1.0)
        self.MIN_STEERING_EFFORT = self.get_parameter('~limits/steering/min', -1.0)
        self.MAX_STEERING_EFFORT = self.get_parameter('~limits/steering/max',  1.0)

        self.PUBLISH_EARLY = self.get_parameter('~open_loop/publish_early', True)

        # Make closed loop velocity controller
        controller_type = self.get_parameter('~controller_type')
        if controller_type == 'lead_lag':
            self.controller = LeadLagController(
                self.get_parameter('~lead/gain'),
                self.get_parameter('~lead/alpha'),
                self.get_parameter('~lead/beta'),
                self.get_parameter('~lag/alpha'),
                self.get_parameter('~lag/beta'))
        elif controller_type == 'pid':
            self.controller = PIDController(
                self.get_parameter('~pid/kp'),
                self.get_parameter('~pid/ki'),
                self.get_parameter('~pid/kd'),
                self.get_parameter('~pid/anti_windup'),
                self.get_parameter('~pid/deadband'),
                1.0 / self.get_parameter('~rate', 10.0))
        elif controller_type == 'none':
            self.controller = None
        else:
            raise NotImplementedError(f"unrecognized controller type: {controller_type}")

        # State variables
        self.vr_ref = None
        self.psi_ref = None

        # Publishers
        self.pub = self.create_publisher('effort', ControlEffort, 1)

        # Subscribers
        self.command_subscriber = self.create_subscription('command', ControlCommand, self.command_cb)
        self.odom_subscriber = self.create_subscription('odom', Odometry, self.odom_cb)


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
        msg.header.stamp = rclpy.Time.now()
        msg.throttle = throttle_effort
        msg.steering = steering_effort
        self.pub.publish(msg)

def main(args=None):
    # Initializes ROS 2 Communications (Between Nodes)
    rclpy.init(args=args)

    node = ControllerNode()         # Create our custom node
    rclpy.spin(node)                # Spin Node (Keep it Alive & Running)    
    rclpy.shutdown()                # Ends ROS 2 Communications

# Allows us to run / execute our node directly
if __name__ == "__main__":
    main()


# -------- Define Two Possible Controllers -------- #

class PIDController:
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


class LeadLagController:
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


