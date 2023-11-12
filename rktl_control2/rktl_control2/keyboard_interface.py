#!/usr/bin/env python3
"""
Allows for controlling the car using a keyboard interface. Uses the terminal
(with ncurses) as an interface.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rclpy
from rktl_msgs.msg import ControlEffort
from std_srvs.srv import Empty

from curses import wrapper
from time import sleep

def main(win):
    win.nodelay(True)
    throttle_effort = 0
    steering_effort = 0
    while not rclpy.ok():
        key = ""
        try:
            key = win.getkey()
        except:
            pass
        if key == 'q':
            exit()
        elif key == 'w':
            throttle_effort += 0.05
        elif key == 's':
            throttle_effort -= 0.05
        elif key == 'a':
            steering_effort += 0.25
        elif key == 'd':
            steering_effort -= 0.25
        elif key == 'r':
            reset_srv.call()
        elif key == 'e':
            throttle_effort = 0
            steering_effort = 0

        throttle_effort = max(min(throttle_effort, 1.0), -1.0)
        steering_effort = max(min(steering_effort, 1.0), -1.0)

        msg = ControlEffort()
        msg.header.stamp = rclpy.Time.now()
        msg.throttle = throttle_effort
        msg.steering = steering_effort
        effort_pub.publish(msg)

        win.erase()
        win.addstr("keyboard controller\n")
        win.addstr("WASD for throttle / steering\n")
        win.addstr(f"current throttle: {throttle_effort:0.2f}\n")
        win.addstr(f"current steering: {steering_effort:0.2f}\n")
        win.addstr("e to reset throttle / steering to zero\n")
        win.addstr("r to reset sim\n")
        win.addstr("q to quit")
        sleep(0.01)

if __name__ == "__main__":
    node = rclpy.create_node('keyboard')
    effort_pub = node.create_publisher('effort', ControlEffort, queue_size=1)
    reset_srv = node.create_client(Empty, '/sim_reset')

    wrapper(main)
