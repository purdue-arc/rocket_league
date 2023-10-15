import os
import sys

import launch
import launch_ros.actions
#WIP

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='agent_name',
            default_value='agent0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch.actions.GroupAction(
            actions=[
                launch_ros.actions.PushRosNamespace("agents/" + launch.substitutions.LaunchConfiguration("agent_name")),
            ]
            # launch_ros.actions.Node(
            #     package="rktl_planner"
            #     name="path_follower"
            #     executable="path_follower"
            #     output="screen"
            #     parameters=[
            #         {
            #             "car_name": launch.substitutions.LaunchConfiguration("car_name")
            #         }
            #     ]
            # )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
