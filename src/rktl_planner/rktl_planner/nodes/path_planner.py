#!/usr/bin/env python3

import math
import rclpy
import numpy as np
from rktl_dependencies.transformations import quaternion_from_euler, euler_from_quaternion
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from rktl_msgs.msg import BezierPathList, Path
from std_srvs.srv import Empty, Empty_Request, Empty_Response
from rktl_planner.srv import CreateBezierPath, CreateBezierPathRequest

def create_simple_path_req(car_odom, ball_odom, goal_pos):
    req = CreateBezierPathRequest()

    req.velocity = 0.5
    req.bezier_segment_duration.data = node.create_rate(0.5)
    req.linear_segment_duration.data = node.create_rate(0.01)

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = car_odom.pose.pose.orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=node.create_rate(5.0)).to_msg())

    # Target 1 (ball pos)
    final_vec_x = goal_pos[0] - ball_odom.pose.pose.position.x
    final_vec_y = goal_pos[1] - ball_odom.pose.pose.position.y
    final_quat = quaternion_from_euler(
        0., 0., math.atan2(final_vec_y, final_vec_x))
    final_vec_len = math.sqrt(final_vec_x ** 2 + final_vec_y ** 2)
    pose1 = Pose()
    pose1.position.x = ball_odom.pose.pose.position.x
    pose1.position.y = ball_odom.pose.pose.position.y
    pose1.position.z = 0.0
    pose1.orientation.x = final_quat[0]
    pose1.orientation.y = final_quat[1]
    pose1.orientation.z = final_quat[2]
    pose1.orientation.w = final_quat[3]
    req.target_poses.append(pose1)
    req.target_durations.append(Duration(data=node.create_rate(1.0)).to_msg())

    # Target 2 (stop)
    pose2 = Pose()
    pose2.position.x = pose1.position.x + final_vec_x / final_vec_len / 2
    pose2.position.y = pose1.position.y + final_vec_y / final_vec_len / 2
    pose2.position.z = 0.0
    pose2.orientation = pose2.orientation
    req.target_poses.append(pose2)

    return req

def create_backup_path_req(car_odom, ball_odom, goal_pos):
    req = CreateBezierPathRequest()
    req.velocity = -0.5
    req.bezier_segment_duration.data = node.create_rate(0.5)
    req.linear_segment_duration.data = node.create_rate(0.01)

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = car_odom.pose.pose.orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=node.create_rate(1.0)).to_msg())

    # Target 1 (in front of ball)
    final_vec_x = goal_pos[0] - ball_odom.pose.pose.position.x
    final_vec_y = goal_pos[1] - ball_odom.pose.pose.position.y
    final_quat = quaternion_from_euler(
        0., 0., math.atan2(final_vec_y, final_vec_x))
    pose1 = Pose()
    pose1.position.x = ball_odom.pose.pose.position.x - final_vec_x
    pose1.position.y = ball_odom.pose.pose.position.y - final_vec_y
    pose1.position.z = 0.0
    pose1.orientation.x = final_quat[0]
    pose1.orientation.y = final_quat[1]
    pose1.orientation.z = final_quat[2]
    pose1.orientation.w = final_quat[3]
    req.target_poses.append(pose1)

    return req

def create_complex_path_req(car_odom, ball_odom, goal_pos):
    car_orient = car_odom.pose.pose.orientation
    car_yaw = euler_from_quaternion(
        np.array([car_orient.x, car_orient.y, car_orient.z, car_orient.w]))[2]
    car_x = car_odom.pose.pose.position.x
    ball_x = ball_odom.pose.pose.position.x
    if (car_yaw < math.pi/2 or car_yaw > 3*math.pi/2) and (car_x > ball_x):
        return create_backup_path_req(car_odom, ball_odom, goal_pos)
    else:
        return create_simple_path_req(car_odom, ball_odom, goal_pos)

class PathPlanner(object):
    def __init__(self):
        rclpy.init()
        global node
        node = rclpy.create_node('path_planner')

        planner_type = node.declare_parameter('~planner_type', 'simple')
        if planner_type == 'simple':
            self.path_req = create_simple_path_req
        elif planner_type == 'complex':
            self.path_req = create_complex_path_req
        else:
            raise NotImplementedError(f'unrecognized planner type: {node.declare_parameter("~planner_type")}')

        # Subscribers
        car_name = node.declare_parameter('~car_name')
        node.create_subscription(Odometry, f'/cars/{car_name}/odom', self.car_odom_cb, rclpy.qos.QoSProfile())
        node.create_subscription(Odometry, '/ball/odom', self.ball_odom_cb)

        # Services
        self.reset_server = node.create_service(Empty, 'reset_planner', self.reset)
        self.path_client = node.create_client(CreateBezierPath, 'create_bezier_path')

        # Publishers
        self.linear_path_pub = node.create_publisher(Path, 'linear_path', 1, latch=True)
        self.bezier_path_pub = node.create_publisher(BezierPathList, 'bezier_path', 1, latch=True)

        self.car_odom = Odometry()
        self.ball_odom = Odometry()

        self.goal_pos = (node.declare_paramete('/field/length', 1) / 2, 0.)

        rclpy.spin()

    def car_odom_cb(self, data: Odometry):
        self.car_odom = data

    def ball_odom_cb(self, data: Odometry):
        self.ball_odom = data

    def reset(self, _: EmptyRequest):
        req = self.path_req(self.car_odom, self.ball_odom, self.goal_pos)
        res = self.path_client(req)
        if self.linear_path_pub:
            self.linear_path_pub.publish(res.linear_path)
        if self.bezier_path_pub:
            bezier_path_list = BezierPathList()
            bezier_path_list.paths = res.bezier_paths
            self.bezier_path_pub.publish(bezier_path_list)
        return EmptyResponse()

if __name__ == '__main__':
    PathPlanner()
