#!/usr/bin/env python3
"""Contains the PathPlanner Node.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# 3rd party modules
import rclpy
#import rospy
import numpy as np
import math
import time
from nav_msgs.msg import Odometry

# Local modules
import rktl_planner.convert as convert
import rktl_planner.pure_pursuit as pursuit
from rktl_msgs.msg import Path, ControlCommand


class PathFollower(object):
    """A node to ensure the car follows a given trajectory."""

    def __init__(self):
        rclpy.init()
        global node
        node = rclpy.Node('path_follower')

        self.path_start_time = None
        self.path = None
        self.last_pose_idx = None
        self.start_time = None
        self.max_speed = None

        self.frame_id = node.declare_parameter('~frame_id', 'map')

        # Max speed to travel path
        self.max_speed = node.declare_parameter('~max_speed', 0.1)

        # Radius to search for intersections
        self.lookahead_dist = node.declare_parameter('~lookahead_dist', 0.15)

        # Coeffient to adjust lookahead distance by speed
        self.lookahead_gain = node.declare_parameter('~lookahead_gain', 0.035)

        # Number of waypoints to search per pass (-1 is full path)
        self.lookahead_pnts = node.declare_parameter('~lookahead_pnts', -1)

        # Publishers
        car_name = node.declare_parameter('~car_name')
        self.bot_velocity_cmd = node.create_publisher(ControlCommand, f'/cars/{car_name}/command', 1)

        # Subscribers
        node.create_subscription(Odometry, f'/cars/{car_name}/odom', self.odom_cb, qos_profile=1)
        node.create_subscription(Path, 'linear_path', self.path_cb, qos_profile=1)

        rclpy.spin(node)

    def path_cb(self, path_msg: Path):
        """Creates path using waypoints in Path message."""
        self.path_start_time = path_msg.header.stamp
        self.goal_vel = path_msg.velocity
        self.path = path_msg.waypoints
        self.last_pnt_idx = 0
        self.start_time = time.time()

    def odom_cb(self, odom_msg: Odometry):
        """Updates car odometry and follows current path."""
        if self.path:
            if self.last_pose_idx == None:
                self.last_pose_idx = 0

            # Converting odom msg to numpy arrays
            bot_pos, bot_orient, _ = convert.pose_covar_to_array(odom_msg.pose)
            bot_linear, bot_angular, _ = convert.twist_covar_to_array(
                odom_msg.twist)

            # Set lookahead dist by lookahead gain and current speed
            lookahead_boost = np.linalg.norm(bot_linear) * self.lookahead_gain.get_parameter_value().double_value
            lookahead_dist = rclpy.Parameter('~lookahead_dist', rclpy.Parameter.Type.DOUBLE, self.lookahead_dist.get_parameter_value().double_value + lookahead_boost)
            
            # Set number of waypoints to check
            if self.lookahead_pnts.get_parameter_value().integer_value == -1:
                lookahead_pnts = len(self.path)
            else:
                lookahead_pnts = self.lookahead_pnts.get_parameter_value().integer_value

            # Find next valid intersection along path
            intersect = None
            goal_vel = self.goal_vel
            i = self.last_pose_idx
            pnts_checked = 0
            while pnts_checked < lookahead_pnts:
                if i == len(self.path) - 1:
                    i = 0

                start_pos, _ = convert.pose_to_array(self.path[i].pose)
                end_pos, _ = convert.pose_to_array(self.path[i + 1].pose)

                path_seg = end_pos - start_pos
                bot_path = start_pos - bot_pos

                intersect = pursuit.find_intersection(path_seg, bot_path,
                    lookahead_dist)

                if intersect is not None:
                    self.last_pose_idx = i
                    intersect += start_pos

                    # Check if intersection is behind vehicle
                    d_angle = pursuit.calculate_angle(intersect, bot_pos,
                        bot_orient, lookahead_dist, goal_vel < 0)
                    if abs(d_angle) > math.pi/2:                  
                        intersect = None
                    else:
                        break

                i += 1
                pnts_checked += 1

            # If no intersection found, stop moving
            if intersect is None:
                self.bot_velocity_cmd.publish(ControlCommand())
                node.get_logger().warn("No intersection could be found.")
                #rospy.logwarn("No intersection could be found.")
                return

            # Calculate curvature
            turn_rad = pursuit.calculate_turn_rad(intersect, bot_pos,
                bot_orient, lookahead_dist, goal_vel < 0)

            # Publish command data
            cmd = ControlCommand()
            cmd.velocity = goal_vel
            cmd.curvature = -(1.0 / turn_rad)
            self.bot_velocity_cmd.publish(cmd)


if __name__ == "__main__":
    PathFollower()
