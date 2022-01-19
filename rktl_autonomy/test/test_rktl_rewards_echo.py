#!/usr/bin/env python3
"""Tests for rewards in rocket_league_interface.py. This node will echo fake
data back as if it was the sim.
License:
  BSD 3-Clause License
  Copyright (c) 2022, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

# ROS
from http.client import SWITCHING_PROTOCOLS
import rospy
from nav_msgs.msg import Odometry
from rktl_msgs.msg import MatchStatus
from std_srvs.srv import Empty, EmptyResponse
from rosgraph_msgs.msg import Clock

import gym
from threading import Lock

def easy_odom(x, y, v):
    """Helper to construct Odom msgs."""
    odom_msg = Odometry()
    odom_msg.pose.pose.orientation.w = 1
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.twist.twist.linear.x = v
    return odom_msg

class Echo(object):
    def __init__(self):
        rospy.init_node('echo')

        # Publishers
        self.car_pub = rospy.Publisher('car0/odom', Odometry, queue_size=1) 
        self.ball_pub = rospy.Publisher('ball/odom', Odometry, queue_size=1)
        self.stat_pub = rospy.Publisher('match_status', MatchStatus, queue_size=1)

        # Subscribers
        rospy.Subscriber('/clock', Clock, self.clock_cb)

        # Services
        rospy.Service('sim_reset', Empty, self.reset)

        self.ticks = 0

        rospy.spin()

    def clock_cb(self, __):
        """Callback for clock signal."""
        if self.ticks == 0:
            self.car_pub.publish(easy_odom(0, 0, 1))
            self.ball_pub.publish(easy_odom(0, 0, 0))
            self.stat_pub.publish(MatchStatus(status=MatchStatus.ONGOING))
        elif self.ticks == 1:
            self.car_pub.publish(easy_odom(0, 0, -1))
            self.ball_pub.publish(easy_odom(1, 0, 0))
            self.stat_pub.publish(MatchStatus(status=MatchStatus.VICTORY_TEAM_A))
        elif self.ticks == 2:
            self.car_pub.publish(easy_odom(2.3, 0, 0))
            self.ball_pub.publish(easy_odom(0, 0, 0))
            self.stat_pub.publish(MatchStatus(status=MatchStatus.ONGOING))
        elif self.ticks == 3:
            self.car_pub.publish(easy_odom(0, -1.65, 1))
            self.ball_pub.publish(easy_odom(0, 0, 0))
            self.stat_pub.publish(MatchStatus(status=MatchStatus.VICTORY_TEAM_A))
        else:
            exit()
        self.ticks += 1

    def reset(self, __):
        """Do nothing."""
        return EmptyResponse()

if __name__ == "__main__":
    Echo()