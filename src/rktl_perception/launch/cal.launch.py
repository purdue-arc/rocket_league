import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib'
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib'
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib'
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
