import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='device',
            default_value='/dev/input/js0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='delay',
            default_value='0.1'
        ),
        launch.actions.GroupAction(
            actions=[
                launch_ros.actions.PushRosNamespace("cars/" + launch.substitutions.LaunchConfiguration("car_name")),

                launch_ros.actions.Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    output='screen',
                    parameters=[
                        {
                            'dev': launch.substitutions.LaunchConfiguration('device')
                        },
                        {
                            'default_trig_val': 'true'
                        }
                    ]
                ),
                launch_ros.actions.Node(
                    package='rktl_control',
                    executable='xbox_interface',
                    name='xbox_interface',
                    output='screen',
                    parameters=[
                        {
                            'base_throttle': '0.75'
                        },
                        {
                            'boost_throttle': '1.25'
                        },
                        {
                            'cooldown_ratio': '3'
                        },
                        {
                            'max_boost': '2'
                        }
                    ],
                    actions=[
                        launch_ros.actions.SetRemap(
                            src="joy",
                            dst="joy)mux",
                        )
                    ]
                )
            ]
        )

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()