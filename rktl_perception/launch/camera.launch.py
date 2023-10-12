import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='load_manager',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='manager_name',
            default_value='camera_manager'
        ),
        launch.actions.DeclareLaunchArgument(
            name='manager_threads',
            default_value='4'
        ),
        launch.actions.DeclareLaunchArgument(
            name='camera_name',
            default_value='cam0'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
