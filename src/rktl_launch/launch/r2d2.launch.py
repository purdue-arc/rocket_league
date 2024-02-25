import os
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_control',
            executable='pose_synchronizer',
            name='pose_sync_node',
            parameters=[
                get_package_share_directory(
                    'rktl_control') + '/config/pose_synchronizer.yaml'
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_perception'), 'launch/camera.launch.py')
            ),
            launch_arguments={
                'camera_name': 'cam2'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_perception'), 'launch/camera.launch.py')
            ),
            launch_arguments={
                'camera_name': 'cam3'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_launch'), 'launch/rocket_league.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
