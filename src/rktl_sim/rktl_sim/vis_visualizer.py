#!/usr/bin/env python3
"""Node to run the visualizer with ROS bindings.
License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.

TODO:
- Scale to support multiple cars
"""

# 3rd party modules
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import rclpy
import sys
from rclpy.exceptions import ROSInterruptException

from rktl_dependencies.transformations import euler_from_quaternion

from threading import Lock
import numpy as np


# Local library
from rktl_msgs.msg import Path, BezierPathList
from rktl_planner import BezierPath


class VisualizerROS(object):
    """ROS wrapper for the visualizer."""

    def __init__(self):
        #rospy.init_node("visualizer")
        rclpy.init(args=sys.argv)
        global node
        node = rclpy.create_node("visualizer")
        
        # Collecting global parameters
        # field_width = node.get_param('/field/width')
        # field_length = rospy.get_param('/field/length')
        # goal_width = rospy.get_param('/field/goal/width')
        # wall_thickness = rospy.get_param('/field/wall_thickness')
        # ball_radius = rospy.get_param("/ball/radius")
        field_width = node.declare_parameter('/field/width').get_parameter_value().double_value
        field_length = node.declare_parameter('/field/length').get_parameter_value().double_value
        goal_width = node.declare_parameter('/field/goal/width').get_parameter_value().double_value
        wall_thickness = node.declare_parameter('/field/wall_thickness').get_parameter_value().double_value
        ball_radius = node.declare_parameter("/ball/radius").get_parameter_value().double_value



        # Creating pygame render
        
        self.window = visualizer.Window(
            field_width, field_length, wall_thickness,
            node.declare_parameter('~window_name', 'Rocket League Visualizer').value)
        
        # Collecting private parameters
        # self.frame_id = rospy.get_param("~frame_id", "map")
        # self.timeout = rospy.get_param("~timeout", 10)
        # rate = rospy.Rate(rospy.get_param("~rate", 20))
        # car_width = rospy.get_param("~cars/body_width")
        # car_length = rospy.get_param("~cars/body_length")
        self.frame_id = node.declare_parameter("~frame_id", "map").value
        self.timeout = node.declare_parameter("~timeout", 10).value
        rate = rclpy.create_node('simple_node').create_rate((node.declare_parameter("~rate", 20).get_parameter_value().double_value))
        car_width = node.declare_parameter("~cars/body_width").value
        car_length = node.declare_parameter("~cars/body_length").value

        # Setting up field
        id = 0
        # field_img_path = rospy.get_param("~media/field", None)
        field_img_path = node.declare_parameter("~media/field", None).value

        if field_img_path is not None:
            self.window.createAsset(
                id, field_width, field_length, imgPath=field_img_path,
                initPos=(0, 0))
            id += 1

        # Setting up sidewalls
        sidewall_length = field_length + (wall_thickness * 2.0)
        self.window.createAsset(
            id, wall_thickness, sidewall_length, color=(0, 0, 0),
            initPos=(0., (field_width + wall_thickness) / 2.))
        id += 1
        
        self.window.createAsset(
            id, wall_thickness, sidewall_length, color=(0, 0, 0),
            initPos=(0., -(field_width + wall_thickness) / 2.))
        id += 1

        # Setting up backwalls
        backwall_width = (field_width - goal_width) / 2.
        backwall_x = (field_length + wall_thickness) / 2.
        backwall_y = (field_width / 2.) - (backwall_width / 2.)
        self.window.createAsset(
            id, backwall_width, wall_thickness, color=(0, 0, 0),
            initPos=(backwall_x, backwall_y))
        id += 1

        self.window.createAsset(
            id, backwall_width, wall_thickness, color=(0, 0, 0),
            initPos=(backwall_x, -backwall_y))
        id += 1
    
        self.window.createAsset(
            id, backwall_width, wall_thickness, color=(0, 0, 0),
            initPos=(-backwall_x, backwall_y))
        id += 1

        self.window.createAsset(
            id, backwall_width, wall_thickness, color=(0, 0, 0),
            initPos=(-backwall_x, -backwall_y))
        id += 1

        # Setting up car
        # car_img_path = rospy.get_param("~media/car", None)
        car_img_path = node.declare_parameter("~media/car", None).value


        if car_img_path is not None:
            self.car_id = id
            self.window.createAsset(
                self.car_id, car_width, car_length, imgPath=car_img_path)
            id += 1

        # Setting up ball
        # ball_img_path = rospy.get_param("~media/ball", None)
        ball_img_path = node.declare_parameter("~media/ball", None).value

        if ball_img_path is not None:
            self.ball_id = id
            self.window.createAsset(
                self.ball_id, ball_radius * 2, ball_radius * 2, imgPath=ball_img_path)
            id += 1

        # Setting up goals
        self.goal1_id = id
        self.window.createAsset(
            self.goal1_id, goal_width, wall_thickness, color=(255, 255, 255))
        self.window.updateAssetPos(
            self.goal1_id, (field_length / 2) + (wall_thickness / 2), 0)
        id += 1
        
        self.goal2_id = id
        self.window.createAsset(
            self.goal2_id, goal_width, wall_thickness, color=(255, 255, 255))
        self.window.updateAssetPos(
            self.goal2_id, -((field_length / 2) + (wall_thickness / 2)), 0)
        id += 1

        self.lines_id = 11
        self.window.createAsset(self.lines_id, 0, 0, color=(255, 0, 0), lines=True)

        self.circle_id = 12
        self.window.createAsset(self.circle_id, 0, 0, circle=True, color=(255,0,0), radius=0)

        self.lock = Lock()
        self.last_time = None
        self.path = None

        # Subscribers
        # rospy.Subscriber("/ball/odom", Odometry, self.ball_odom_cb)
        # rospy.Subscriber("/cars/car0/odom", Odometry, self.car_odom_cb)
        # rospy.Subscriber("/cars/car0/path", Path, self.path_arr_cb)
        # rospy.Subscriber("/cars/car0/lookahead_pnt", Float32, self.lookahead_cb)
        # rospy.Subscriber("/agents/agent0/bezier_path", BezierPathList, self.bezier_path_cb)

        node.create_subscription(Odometry, "/ball/odom", self.ball_odom_cb, qos_profile=1)
        node.create_subscription(Odometry, "/cars/car0/odom", self.car_odom_cb, qos_profile=1)
        node.create_subscription(Path, "/cars/car0/path", self.path_arr_cb, qos_profile=1)
        node.create_subscription(Float32, "/cars/car0/lookahead_pnt", self.lookahead_cb, qos_profile=1)
        node.create_subscription(BezierPathList, "/agents/agent0/bezier_path", self.bezier_path_cb, qos_profile=1)
        

        while not rclpy.ok():
            try:
                self.window.show()
            except self.window.ShutdownError:
                exit()
            try:
                rate.sleep()
            except ROSInterruptException:
                pass


    def car_odom_cb(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orient = odom_msg.pose.pose.orientation
        quat = [orient.x, orient.y, orient.z, orient.w]
        heading = euler_from_quaternion(quat)[2]
        heading = heading * 180. / math.pi
        self.window.updateAssetPos(self.car_id, x, y)
        self.window.updateAssetAngle(self.car_id, heading)
        self.window.updateAssetPos(self.circle_id, x, y)

    def ball_odom_cb(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        self.window.updateAssetPos(self.ball_id, x, y)

    def path_arr_cb(self, path_msg: Path):
        """Callback for path array messages."""
        self.window.resetAssetLines(self.lines_id)
        self.path = path_msg.waypoint
        for point in self.path:
            x = point.pose.position.x
            y = point.pose.position.y
            self.window.updateAssetPos(self.lines_id, x, y)
    
    def bezier_path_cb(self, msg: BezierPathList):
        """Callback for bezier path messages."""
        self.window.resetAssetLines(self.lines_id)
        paths = [BezierPath(x) for x in msg.paths]
        for path in paths:
            #for point in path.bezier_curve.control_points:
            #    self.window.updateAssetPos(self.lines_id, point.x, point.y)
            sec = path.duration.to_sec()
            for t in np.linspace(0., sec, int(50 * sec + 0.5)):
                point = path.at(t)
                self.window.updateAssetPos(self.lines_id, point.x, point.y)

    def lookahead_cb(self, msg: Float32):
        """Callback for lookahead_pnt messages."""
        self.window.updateAssetRadius(self.circle_id, msg.data)

def main():
    VisualizerROS()

if __name__ == "__main__":
    main()
