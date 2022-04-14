#!/usr/bin/env python3

import math
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, \
    euler_from_quaternion, quaternion_multiply
from std_msgs.msg import Duration
from geometry_msgs.msg import Pose
from rktl_planner.srv import CreateBezierPathRequest

def create_score_path_req(car_odom, ball_odom, goal_pos, bkw=False):
    req = CreateBezierPathRequest()

    req.velocity = 0.5
    if bkw:
        req.velocity = -0.5

    req.bezier_segment_duration.data = rospy.Duration(0.5)
    req.linear_segment_duration.data = rospy.Duration(0.01)

    orientation = car_odom.pose.pose.orientation
    if bkw:
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        x, y, z, w = quaternion_multiply([x, y, z, w], [0, 0, 0, 1])
        orientation.x = x
        orientation.y = y
        orientation.z = z
        orientation.w = w

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=rospy.Duration(5.0)))

    # Target 1 (ball pos)
    final_vec_x = goal_pos[0] - ball_odom.pose.pose.position.x
    final_vec_y = goal_pos[1] - ball_odom.pose.pose.position.y
    final_heading = math.atan2(final_vec_y, final_vec_x)
    if bkw:
        final_heading += math.pi

    final_quat = quaternion_from_euler(
        0., 0., final_heading)
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
    req.target_durations.append(Duration(data=rospy.Duration(1.0)))

    # Target 2 (stop)
    pose2 = Pose()
    pose2.position.x = pose1.position.x + final_vec_x / final_vec_len / 2
    pose2.position.y = pose1.position.y + final_vec_y / final_vec_len / 2
    pose2.position.z = 0.0
    pose2.orientation = pose1.orientation
    req.target_poses.append(pose2)

    return req

def create_backup_path_req(car_odom, goal_pos, bkw=True):
    req = CreateBezierPathRequest()
    req.velocity = -0.5
    if not bkw:
        req.velocity = 0.5
    req.bezier_segment_duration.data = rospy.Duration(0.5)
    req.linear_segment_duration.data = rospy.Duration(0.01)

    orientation = car_odom.pose.pose.orientation
    if not bkw:
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        x, y, z, w = quaternion_multiply([x, y, z, w], [0, 0, 0, 1])
        orientation.x = x
        orientation.y = y
        orientation.z = z
        orientation.w = w

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=rospy.Duration(1.0)))

    # Target 1 (in front of goal)
    final_vec_x = 0.5
    final_vec_y = 0
    final_quat = quaternion_from_euler(
        0., 0., math.atan2(final_vec_y, final_vec_x))
    pose1 = Pose()
    pose1.position.x = car_odom.pose.pose.position.x - 1.0
    pose1.position.y = car_odom.pose.pose.position.y
    pose1.position.z = 0.0
    pose1.orientation.x = final_quat[0]
    pose1.orientation.y = final_quat[1]
    pose1.orientation.z = final_quat[2]
    pose1.orientation.w = final_quat[3]
    req.target_poses.append(pose1)

    return req

def create_dislodge_path_req(car_odom, ball_odom, bkw=False):
    req = CreateBezierPathRequest()
    req.velocity = 0.5
    if bkw: 
        req.velocity = -0.5
    req.bezier_segment_duration.data = rospy.Duration(0.5)
    req.linear_segment_duration.data = rospy.Duration(0.01)

    orientation = car_odom.pose.pose.orientation
    if bkw:
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        x, y, z, w = quaternion_multiply([x, y, z, w], [0, 0, 0, 1])
        orientation.x = x
        orientation.y = y
        orientation.z = z
        orientation.w = w

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=rospy.Duration(1.0)))

    # Target 1 (ball pos)
    final_vec_x = 0.5
    final_vec_y = 0
    final_vec_len = math.sqrt(final_vec_x ** 2 + final_vec_y ** 2)
    final_quat = quaternion_from_euler(
        0., 0., math.atan2(final_vec_y, final_vec_x))
    pose1 = Pose()
    pose1.position.x = ball_odom.pose.pose.position.x
    pose1.position.y = ball_odom.pose.pose.position.y
    pose1.position.z = 0.0
    pose1.orientation.x = final_quat[0]
    pose1.orientation.y = final_quat[1]
    pose1.orientation.z = final_quat[2]
    pose1.orientation.w = final_quat[3]
    req.target_durations.append(Duration(data=rospy.Duration(1.0)))
    req.target_poses.append(pose1)

    # Target 2 (stop)
    pose2 = Pose()
    pose2.position.x = pose1.position.x + final_vec_x / final_vec_len / 2
    pose2.position.y = pose1.position.y + final_vec_y / final_vec_len / 2
    pose2.position.z = 0.0
    pose2.orientation = pose2.orientation
    req.target_poses.append(pose2)

    return req

def create_turn_path_req(car_odom):
    req = CreateBezierPathRequest()
    req.velocity = -0.5
    req.bezier_segment_duration.data = rospy.Duration(0.5)
    req.linear_segment_duration.data = rospy.Duration(0.01)

    # Target 0 (car pos)
    pose0 = Pose()
    pose0.position.x = car_odom.pose.pose.position.x
    pose0.position.y = car_odom.pose.pose.position.y
    pose0.position.z = 0.0
    pose0.orientation = car_odom.pose.pose.orientation
    req.target_poses.append(pose0)
    req.target_durations.append(Duration(data=rospy.Duration(1.0)))

    # Target 1 (ball pos)
    final_vec_x = 0.0
    final_vec_y = 0.5
    final_vec_len = math.sqrt(final_vec_x ** 2 + final_vec_y ** 2)
    final_quat = quaternion_from_euler(
        0., 0., math.atan2(final_vec_y, final_vec_x))
    pose1 = Pose()
    pose1.position.x = 0.0
    pose1.position.y = 1.0
    pose1.position.z = 0.0
    pose1.orientation.x = final_quat[0]
    pose1.orientation.y = final_quat[1]
    pose1.orientation.z = final_quat[2]
    pose1.orientation.w = final_quat[3]
    req.target_poses.append(pose1)

    return req