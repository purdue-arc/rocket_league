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
            name='render',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='weights',
            default_value='~/catkin_ws/data/snake/weights'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rate',
            default_value='10.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='snake_size',
            default_value='7'
        ),
        launch.actions.DeclareLaunchArgument(
            name='arena_size',
            default_value='10'
        ),
        launch_ros.actions.Node(
            package='snakesim',
            executable='snake_node',
            name='snake_env',
            output='screen',
            parameters=[
                {
                    'render': launch.substitutions.LaunchConfiguration('render')
                },
                {
                    'snake/initial_segments': launch.substitutions.LaunchConfiguration('snake_size')
                },
                {
                    'snake/growth': '0'
                },
                {
                    'arena/bounds': launch.substitutions.LaunchConfiguration('arena_size')
                },
                {
                    'rate': launch.substitutions.LaunchConfiguration('rate')
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_autonomy',
            executable='snake_agent',
            name='snake_agent',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'num_segments': launch.substitutions.LaunchConfiguration('snake_size')
                },
                {
                    'field_size': launch.substitutions.LaunchConfiguration('arena_size')
                },
                {
                    'weights': launch.substitutions.LaunchConfiguration('weights')
                },
                get_package_share_directory(
                    'rktl_autonomy') + '/config/snake.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_autonomy',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[
                get_package_share_directory(
                    'rktl_autonomy') + '/config/snake.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
