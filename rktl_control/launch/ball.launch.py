import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_control',
            executable='mean_odom_filter',
            name='mean_odom_filter',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'rktl_control') + '/config/mean_odom_filter.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
