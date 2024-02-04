#!/usr/bin/env python3
"""
Downsample the raw perception data to produce synchronized poses.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# ROS
import rclpy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
from rktl_dependencies.transformations import euler_from_quaternion, quaternion_from_euler

from collections import deque
from math import sin, cos, atan2


class PoseSynchronizer(object):
    """Class to synchronize and buffer all poses."""

    def __init__(self):
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('pose_synchronizer')

        # constants
        self.MAP_FRAME = self.node.declare_parameter('~map_frame', 'map')
        self.TOPICS = self.node.declare_parameter('~topics')
        self.PERIOD = rclpy.time.Time(1/self.node.declare_parameter('~rate', 10.0))
        self.DELAY = rclpy.time.Time(self.node.declare_parameter('~delay', 0.15))
        self.PUB_LATENCY = self.node.declare_parameter('~publish_latency', False)
        self.USE_WEIGHTS = {
            topic: self.node.declare_parameter('~use_weights')[i] for i,
            topic in enumerate(self.TOPICS)}

        # variables
        self.buffers = {
            topic: deque(maxlen=self.node.declare_parameter('buffer_size', 15))
            for topic in self.TOPICS}
        self.pubs = {
            topic: self.node.create_publisher(PoseWithCovarianceStamped,
                topic + '_sync', queue_size=1)
            for topic in self.TOPICS}
        if self.PUB_LATENCY:
            self.latency_pubs = {
                topic: self.node.create_publisher(Float32,
                    topic + '_sync_latency', queue_size=1)
                for topic in self.TOPICS}
        for topic in self.TOPICS:
            self.node.create_subscription(PoseWithCovarianceStamped,
                topic, lambda msg,
                topic=topic: self.recv_pose(topic, msg))

        # main loop
        rate = self.node.create_rate(1/self.PERIOD.to_sec())
        while rclpy.ok():
            now = self.node.get_clock().now().to_msg()()
            for topic in self.TOPICS:
                self.send_pose(topic, now)
            try:
                rate.sleep()
            except rclpy.ROSInterruptException:
                pass

    def recv_pose(self, topic, pose_msg):
        """Callback for receiving new poses."""
        assert pose_msg.header.frame_id == self.MAP_FRAME
        # extract data
        t = pose_msg.header.stamp
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        weight = pose_msg.pose.pose.position.z
        __, __, yaw = euler_from_quaternion([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
        ])
        # override weight if not being used
        if not self.USE_WEIGHTS[topic]:
            weight = 1.0
        # append to buffer (thread safe)
        sample = (t, x, y, yaw, weight)
        self.buffers[topic].append(sample)
        latency = (self.node.get_clock().now().to_msg() - t).to_sec()
        self.node.get_logger().info("%s: received with %0.3f sec delay", topic, latency)
        if self.PUB_LATENCY:
            self.latency_pubs[topic].publish(latency)

    def send_pose(self, topic, now):
        """Callback for processing received poses and publishing"""
        avg_x = 0.0
        avg_y = 0.0
        avg_hx = 0.0
        avg_hy = 0.0
        total_weight = 0.0
        samples = 0

        # sum and count usable measurements in buffer
        for _ in range(len(self.buffers[topic])):
            t, x, y, yaw, weight = self.buffers[topic].popleft()
            # check if too old
            if t < now - self.DELAY - self.PERIOD/2:
                self.node.get_logger().debug(
                    "%s: discarding pose from %0.3f sec ago", topic,
                    (now - t).to_sec())
            # check if too new
            elif t > now - self.DELAY + self.PERIOD/2:
                self.buffers[topic].append((t, x, y, yaw, weight))
                self.node.get_logger().debug(
                    "%s: skipping pose from %0.3f sec ago", topic, (now - t).to_sec())
            # add to averages
            else:
                avg_x += x * weight
                avg_y += y * weight
                avg_hx += cos(yaw) * weight
                avg_hy += sin(yaw) * weight
                total_weight += weight
                samples += 1

        # check there is sufficient data
        if samples == 0:
            self.node.get_logger().warn(1.0, "%s: insufficient pose data", topic)
            return

        # take avg
        avg_x /= total_weight
        avg_y /= total_weight
        avg_yaw = atan2(avg_hy, avg_hx)
        self.node.get_logger().info("%s: using %d samples", topic, samples)

        # publish message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = now - self.DELAY
        pose_msg.header.frame_id = self.MAP_FRAME
        pose_msg.pose.pose.position.x = avg_x
        pose_msg.pose.pose.position.y = avg_y
        x, y, z, w = quaternion_from_euler(0, 0, avg_yaw)
        pose_msg.pose.pose.orientation.x = x
        pose_msg.pose.pose.orientation.y = y
        pose_msg.pose.pose.orientation.z = z
        pose_msg.pose.pose.orientation.w = w
        self.pubs[topic].publish(pose_msg)


if __name__ == "__main__":
    PoseSynchronizer()
