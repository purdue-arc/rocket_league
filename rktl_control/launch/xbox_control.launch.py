import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='device',
            default_value='/dev/input/js0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='delay',
            default_value='0.1'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
