import os
import sys

import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='ball_color',
            name='ball_color',
            namespace = 'cams/cam0',
            output = 'screen'
        )
        #,
        # launch_ros.actions.Node(
        #     package='rktl_perception',
        #     executable='ball_color',
        #     name='ball_color',
        #     namespace = 'cams/cam1',
        #     output = 'screen'
        # ),
        # launch_ros.actions.Node(
        #     package='rktl_perception',
        #     executable='ball_color',
        #     name='ball_color',
        #     namespace = 'cams/cam2',
        #     output = 'screen'
        # ),
        # launch_ros.actions.Node(
        #     package='rktl_perception',
        #     executable='ball_color',
        #     name='ball_color',
        #     namespace = 'cams/cam3',
        #     output = 'screen'
        # ),
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()