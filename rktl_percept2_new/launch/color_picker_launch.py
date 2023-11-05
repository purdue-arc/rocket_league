import os
import sys

import launch
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package="rktl_perception",
            executable="ball_color",
            namespace="cams/cam0",
            name="ball_color"
        )
        # ,
        # Node(
        #     package="rktl_perception",
        #     executable="ball_color",
        #     namespace="cams/cam1",
        #     name="ball_color"
        # ),
        # Node(
        #     package="rktl_perception",
        #     executable="ball_color",
        #     namespace="cams/cam2",
        #     name="ball_color"
        # ),
        # Node(
        #     package="rktl_perception",
        #     executable="ball_color",
        #     namespace="cams/cam3",
        #     name="ball_color"
        # )
    ]

    return launch.LaunchDescription(nodes)