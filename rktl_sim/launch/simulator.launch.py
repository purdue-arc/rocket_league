import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='pybullet_render',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_mode',
            default_value='realistic'
        ),
        launch_ros.actions.Node(
            package='rktl_sim',
            executable='simulator',
            name='simulator',
            output='screen',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'mode': launch.substitutions.LaunchConfiguration('sim_mode')
                },
                {
                    'render': launch.substitutions.LaunchConfiguration('pybullet_render')
                },
                {
                    'urdf/ball': get_package_share_directory('rktl_sim') + '/urdf/ball.urdf'
                },
                {
                    'urdf/car': get_package_share_directory('rktl_sim') + '/urdf/car.urdf'
                },
                {
                    'urdf/goal': get_package_share_directory('rktl_sim') + '/urdf/goal.urdf'
                },
                {
                    'urdf/sidewall': get_package_share_directory('rktl_sim') + '/urdf/sidewall.urdf'
                },
                {
                    'urdf/backwall': get_package_share_directory('rktl_sim') + '/urdf/backwall.urdf'
                },
                {
                    'urdf/plane': get_package_share_directory('rktl_sim') + '/urdf/plane.urdf'
                },
                get_package_share_directory(
                    'rktl_sim') + '/config/simulation.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
