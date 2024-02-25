import os
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_perception'), 'launch/camera.launch.py')
            ),
            launch_arguments={
                'camera_name': 'cam0'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_perception'), 'launch/camera.launch.py')
            ),
            launch_arguments={
                'camera_name': 'cam1'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
