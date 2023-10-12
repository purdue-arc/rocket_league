import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='plot_log',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='weights_dir',
            default_value='~/catkin_ws/data/rocket_league/'
        ),
        launch.actions.DeclareLaunchArgument(
            name='weights_name',
            default_value='model'
        ),
        launch_ros.actions.Node(
            package='rktl_autonomy',
            executable='rocket_league_agent',
            name='rocket_league_agent',
            output='screen',
            parameters=[
                {
                    'weights': '$(eval weights_dir + weights_name)'
                },
                get_package_share_directory(
                    'rktl_autonomy') + '/config/rocket_league.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_autonomy',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'rktl_autonomy') + '/config/rocket_league.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
