import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='agent0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
