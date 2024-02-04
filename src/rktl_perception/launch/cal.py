import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace='cams/cam0',
            arguments=['--size', '8x6', '--square', '0.026', 'image:=/cams/cam0/image_raw', 'camera:=/cams/cam0']
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace='cams/cam1',
            arguments=['--size', '8x6', '--square', '0.026', 'image:=/cams/cam1/image_raw', 'camera:=/cams/cam1']
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace='cams/cam2',
            arguments=['--size', '8x6', '--square', '0.026', 'image:=/cams/cam2/image_raw', 'camera:=/cams/cam2']
        ),
        Node(
            package='camera_calibration',
            executable='cameracalibrator.py',
            name='calib',
            namespace='cams/cam3',
            arguments=['--size', '8x6', '--square', '0.026', 'image:=/cams/cam3/image_raw', 'camera:=/cams/cam3']
        )
    ])