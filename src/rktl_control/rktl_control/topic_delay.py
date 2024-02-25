#!/usr/bin/env python3
"""
Delay an arbitrary ROS topic without modifying message

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rclpy
import sys
import roslib.message
from collections import deque
from time import sleep
import sys


class Delay(object):
    """Class to delay arbitrary messages."""

    def __init__(self):
        # get args
        assert len(sys.argv) >= 5
        input_name = sys.argv[1]
        output_name = sys.argv[2]
        topic_type = sys.argv[3]
        delay = float(sys.argv[4])

        # get class of type
        msg_class = roslib.message.get_message_class(topic_type)
        assert msg_class is not None

        # create node
        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('delay', anonymous=True)
        pub = self.node.create_publisher(msg_class, output_name, queue_size=1)
        self.node.create_subscription(msg_class, input_name, self.msg_cb)
        self.buffer = deque()

        while rclpy.ok():
            if len(self.buffer) > 0:
                time, msg = self.buffer.popleft()
                wait = delay - (self.node.get_clock().now().to_msg() - time).to_sec()
                if wait > 0:
                    sleep(wait)
                pub.publish(msg)
            else:
                sleep(delay/2)

    def msg_cb(self, msg):
        """Callback for enqueuing new message"""
        self.buffer.append((self.node.get_clock().now().to_msg(), msg))

def main():
    Delay()

if __name__ == "__main__":
    main()
