import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch_ros.actions.Node(
            package='rktl_planner',
            namespace="cars/"+launch.substitutions.LaunchConfiguration("car_name"),
            executable='patrol_planner',
            name='patrol_planner',
            output='screen',
            parameters=[
                get_package_share_directory('rktl_planner') + '/config/patrol_planner.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
