"""
Customizable controller for the car. It implements either a PID controller or a
lead-lag controller.

License:
  BSD 3-Clause License
  Copyright (c) 2023, Autonomous Robotics Club of Purdue (Purdue ARC)
  All rights reserved.
"""

import rclpy
from rclpy.node import Node

# Define our own Controller Node
class ControllerNode(Node):

    # Constructor
    def __init__(self):
        super().__init__("controller") # Name of Node in ROS 2 Communications
        
        self.MAX_SPEED = self.get_param('/cars/throttle/max_speed')
        self.STEERING_THROW = rospy.get_param('/cars/steering/max_throw')
        self.BODY_LENGTH = rospy.get_param('/cars/length')

        self.MIN_THROTTLE_EFFORT = rospy.get_param('~limits/throttle/min', -1.0)
        self.MAX_THROTTLE_EFFORT = rospy.get_param('~limits/throttle/max',  1.0)
        self.MIN_STEERING_EFFORT = rospy.get_param('~limits/steering/min', -1.0)
        self.MAX_STEERING_EFFORT = rospy.get_param('~limits/steering/max',  1.0)

        self.PUBLISH_EARLY = rospy.get_param('~open_loop/publish_early', True)





def main(args=None):
    # Initializes ROS 2 Communications (Between Nodes)
    rclpy.init(args=args)

    # Create our custom node
    node = ControllerNode()

    # Spin Node (Keep it Alive & Running)
    rclpy.spin(node)

    # Ends ROS 2 Communications
    rclpy.shutdown()

# Allows us to run / execute our node directly
if __name__ == "__main__":
    main()
