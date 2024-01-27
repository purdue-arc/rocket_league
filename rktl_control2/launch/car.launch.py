import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_particle_filter',
            default_value='true'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()