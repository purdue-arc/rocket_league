#!/usr/bin/env python3
"""Use ball location to determine when a goal is scored.
License:
  BSD 3-Clause License
  Copyright (c) 2020, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rospy
from nav_msgs.msg import Odometry
from rktl_msgs.msg import Score

def callback():
  # once the ball is detected in the goal, pause car movements, and publish a score
  # only check whether or not ball is in goal while car is enabled
  return 0

rospy.init_node('score_keeper')
effort_pub = rospy.Publisher('score', Score, queue_size=1)
# reset_srv = rospy.ServiceProxy('/sim_reset', Empty)
rospy.Subscriber('/ball/pose', Odometry, callback)
rospy.spin()