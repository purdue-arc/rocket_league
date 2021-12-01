#!/usr/bin/env python3

import unittest
import rospy
import rostest
import sys
from std_msgs.msg import *

class TestSimulation(unittest.TestCase):

  def test_simulation(self):
    rospy.init_node('test_simulation')

    # Wait a while before publishing
    rospy.sleep(rospy.Duration.from_sec(5.0))

    pub = rospy.Publisher("effort/throttle", Float64, latch=True)

    pub.publish(Float64(0.1))

    rospy.sleep(rospy.Duration.from_sec(5.0))
    

if __name__ == '__main__':
  rostest.rosrun('rocket_league_test', 'test_simulation', TestSimulation, sys.argv)