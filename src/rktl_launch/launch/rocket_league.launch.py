import os

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='render',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_type', # Either planner, autonomy, or patrol
            default_value='patrol'
        ),
        launch.actions.DeclareLaunchArgument(
            name='autonomy_weights',
            default_value='model'
        ),
        launch_ros.actions.Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            arguments='--perspective-file ' + os.path.join(get_package_share_directory(
                    'rktl_launch'), 'rqt','rktl.perspective')
        ),
        launch_ros.actions.SetParametersFromFile(
            filename=os.path.join(get_package_share_directory(
                    'rktl_launch'), 'config', 'global_params.yaml')
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_sim'), 'launch/visualizer.launch.py')
            ),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('render'), 'true'))
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_game'), 'launch/game.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/ball.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/car.launch.py')
            ),
            launch_arguments={
                'car_name': 'car0'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_control'), 'launch/hardware_interface.launch.py')
            )
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/simple_agent.launch.py')
            ),
            launch_arguments={
                'agent_name': 'agent0',
                'car_name': 'car0'
            }.items(),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('agent_type'), 'planner'))
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_autonomy'), 'launch/rocket_league/rocket_league_agent.launch.py')
            ),
            launch_arguments={
                'weights_name': launch.substitutions.LaunchConfiguration('autonomy_weights')
            }.items(),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('agent_type'), 'autonomy'))
        ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/patrol_agent.launch.py')
            ),
            launch_arguments={
                'car_name': 'car0'
            }.items(),
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('agent_type'), 'patrol'))
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
