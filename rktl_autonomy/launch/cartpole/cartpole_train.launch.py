import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='rocket_league_agent'
        ),
        launch.actions.DeclareLaunchArgument(
            name='render',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rate',
            default_value='30.0'
        ),
        launch_ros.actions.Node(
            package='rktl_autonomy',
            executable='cartpole_env',
            name='cartpole_env',
            output='screen',
            parameters=[
                {
                    '/use_sim_time': 'true'
                },
                {
                    'frequency': '$(eval 1/rate)'
                },
                {
                    'render': launch.substitutions.LaunchConfiguration('render')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
