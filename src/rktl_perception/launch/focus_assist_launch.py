import os
import sys

import launch
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package="rktl_perception",
            executable="focus_vis",
            namespace="cams/cam0",
            name="focus_vis"
        ),
        Node(
            package="rktl_perception",
            executable="focus_vis",
            namespace="cams/cam1",
            name="focus_vis"
        ),
        Node(
            package="rktl_perception",
            executable="focus_vis",
            namespace="cams/cam2",
            name="focus_vis"
        ),
        Node(
            package="rktl_perception",
            executable="focus_vis",
            namespace="cams/cam3",
            name="focus_vis"
        )
    ]

    return launch.LaunchDescription(nodes)