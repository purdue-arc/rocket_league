import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='plot_log',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='rocket_league_agent'
        ),
        launch.actions.DeclareLaunchArgument(
            name='render',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_mode',
            default_value='ideal'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rate',
            default_value='10.0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_type',
            default_value='none'
        ),
        launch_ros.actions.Node(
            condition=launch.conditions.LaunchConfigurationEquals('plot_log', 'true'),
            package='rktl_autonomy',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                get_package_share_directory('rktl_autonomy') + '/config/rocket_league.yaml'
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rktl_launch'), 'launch/rocket_league_sim.launch.py')
            ),
            launch_arguments={
                'render': launch.substitutions.LaunchConfiguration('render'),
                'sim_mode': launch.substitutions.LaunchConfiguration('sim_mode'),
                'agent_type': launch.substitutions.LaunchConfiguration('agent_type')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
