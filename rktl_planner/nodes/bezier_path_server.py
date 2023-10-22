#!/usr/bin/env python3

#import rospy
import rclpy
import math
from external.transformations import euler_from_quaternion, quaternion_from_euler
from rktl_planner.srv import CreateBezierPath, CreateBezierPathRequest, CreateBezierPathResponse
from rktl_planner.src.rktl_planner import BezierPath
from rktl_msgs.msg import Path as PathMsg, Waypoint as WaypointMsg
from geometry_msgs.msg import Pose, Point, Vector3


def vel(pose: Pose, speed: float):
    q = pose.orientation
    angle = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    head = Vector3()
    head.x = math.cos(angle)
    head.y = math.sin(angle)
    vel = Vector3()
    vel.x = speed * head.x
    vel.y = speed * head.y
    return vel, head


def line(p: Point, v: Vector3, t: float):
    point = Point()
    point.x = p.x + t * v.x
    point.y = p.y + t * v.y
    point.z = p.z + t * v.z
    return point


def create_segment(start_pose: Pose, end_pose: Pose, velocity: float, 
    # duration: rospy.Duration, segment_duration: rospy.Duration):
    duration: rclpy.duration.Duration, segment_duration: rclpy.duration.Duration):
    control_points = [start_pose.position, end_pose.position]
    start_vel, start_head = vel(start_pose, velocity)
    end_vel, end_head = vel(end_pose, velocity)

    t_val = duration.to_sec() / 3

    control_points.insert(1, line(control_points[0], start_vel, t_val))
    control_points.insert(len(control_points) - 1, line(control_points[-1], end_vel, -t_val))
    path = BezierPath(control_points, duration.to_sec())
    num_segments = math.ceil(duration.to_sec() / segment_duration.to_sec())
    segment_length = duration.to_sec() / num_segments
    segments = []
    for i in range(num_segments - 1):
        path0, path1 = path.split(segment_length)
        segments.append(path0)
        path = path1
    segments.append(path)
    return segments


def bezier_path_server(req: CreateBezierPathRequest):
    velocity = req.velocity
    bezier_segments = []
    durations = []
    for i in range(len(req.target_durations)):
        start_pose = req.target_poses[i]
        end_pose = req.target_poses[i + 1]
        duration = req.target_durations[i].data
        segments = create_segment(start_pose, end_pose, velocity,
            duration, req.bezier_segment_duration.data)
        bezier_segments.extend(segments)
        for segment in segments:
            if len(durations) > 0:
                durations.append(durations[-1] + segment.duration.to_sec())
            else:
                durations.append(segment.duration.to_sec())

    res = CreateBezierPathResponse()
    res.bezier_paths = [x.to_msg() for x in bezier_segments]
    res.linear_path = PathMsg()
    res.linear_path.velocity = velocity

    duration = durations[-1]
    segment_length = req.linear_segment_duration.data.to_sec()
    num_segments = math.floor(duration / segment_length)
    idx = 0
    t = 0.
    for i in range(num_segments):
        heading = bezier_segments[idx].angle_at(t)
        quat = quaternion_from_euler(0, 0, heading)
        wp = WaypointMsg()
        wp.pose.position = bezier_segments[idx].at(t)
        wp.pose.orientation.x = quat[0]
        wp.pose.orientation.y = quat[1]
        wp.pose.orientation.z = quat[2]
        wp.pose.orientation.w = quat[3]
        wp.twist.linear = bezier_segments[idx].vel_at(t)
        res.linear_path.waypoints.append(wp)
        t += segment_length
        if t > durations[idx]:
            idx += 1
    return res

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('bezier_path_server')
    service = node.create_service(CreateBezierPath, 'create_bezier_path', bezier_path_server)
    rclpy.spin()