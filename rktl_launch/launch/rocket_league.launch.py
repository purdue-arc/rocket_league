import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='render',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_type',
            default_value='patrol'
        ),
        launch.actions.DeclareLaunchArgument(
            name='autonomy_weights',
            default_value='model'
        ),
        launch_ros.actions.Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_sim'), 'launch/visualizer.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_game'), 'launch/game.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/ball.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/car.launch.py')
            ),
            launch_arguments={
                'car_name': 'car0'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/hardware_interface.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/simple_agent.launch.py')
            ),
            launch_arguments={
                'agent_name': 'agent0',
                'car_name': 'car0'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_autonomy'), 'launch/rocket_league/rocket_league_agent.launch.py')
            ),
            launch_arguments={
                'weights_name': launch.substitutions.LaunchConfiguration('autonomy_weights')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/patrol_agent.launch.py')
            ),
            launch_arguments={
                'car_name': 'car0'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
