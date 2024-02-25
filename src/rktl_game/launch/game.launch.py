import os
import launch
import launch_ros.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_game',
            executable='game_manager',
            name='game_manager',
            output='screen'
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rosbridge_server'), 'launch/rosbridge_websocket.launch.py')
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
