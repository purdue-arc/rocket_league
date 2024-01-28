import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace = 'cams/cam0',
            arguments="--size 8x6 --square 0.026 image:=/cams/cam3/image_raw camera:=/cams/cam0"
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace = 'cams/cam1',
            arguments="--size 8x6 --square 0.026 image:=/cams/cam3/image_raw camera:=/cams/cam1"
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace = 'cams/cam2',
            arguments="--size 8x6 --square 0.026 image:=/cams/cam3/image_raw camera:=/cams/cam2"
        ),
        launch_ros.actions.Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace = 'cams/cam3',
            arguments="--size 8x6 --square 0.026 image:=/cams/cam3/image_raw camera:=/cams/cam3"
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()