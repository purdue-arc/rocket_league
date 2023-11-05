import os
import sys
from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package="rktl_perception",
            executable="projector",
            namespace="cams/cam0",
            name="projector",
            parameters=[{"ground_height": 0.00}]
        ),
        Node(
            package="rktl_perception",
            executable="projector",
            namespace="cams/cam1",
            name="projector",
            parameters=[{"ground_height": -0.02}]
        ),
        Node(
            package="rktl_perception",
            executable="projector",
            namespace="cams/cam2",
            name="projector",
            parameters=[{"ground_height": -0.01}]
        ),
        Node(
            package="rktl_perception",
            executable="projector",
            namespace="cams/cam3",
            name="projector",
            parameters=[{"ground_height": -0.03}]
        ),
        Node(
            package="rviz",
            executable="rviz",
            name="rviz",
            arguments= "-d " + os.path.join(get_package_share_directory('rktl_perception'), 'rviz', 'field.rviz')
        )
    ]

    return launch.LaunchDescription(nodes)