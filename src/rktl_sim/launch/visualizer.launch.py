import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_sim',
            executable='visualizer',
            name='visualizer',
            output='screen',
            parameters=[
                {
                    'media/ball': get_package_share_directory('rktl_sim') + '/media/ball.png'
                },
                {
                    'media/car': get_package_share_directory('rktl_sim') + '/media/car.png'
                },
                {
                    'media/goal': get_package_share_directory('rktl_sim') + '/media/goal.png'
                },
                {
                    'media/field': get_package_share_directory('rktl_sim') + '/media/field.jpg'
                },
                get_package_share_directory(
                    'rktl_sim') + '/config/visualization.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
