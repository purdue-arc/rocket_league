import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rosserial_arduino',
            executable='serial_node.py',
            name='hardware_interface',
            parameters=[
                {
                    launch.substitutions.PathJoinSubstitution([launch_ros.substitutions.FindPackageShare('rktl_control'), '/config/hardware_interface.yaml'])
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()