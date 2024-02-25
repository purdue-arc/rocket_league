import launch
import launch_ros.actions
from launch.substitutions import *
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_particle_filter',
            default_value='true'
        ),
        launch.actions.GroupAction(
            actions=[
                #launch_ros.actions.PushRosNamespace("cars/" + launch.substitutions.LaunchConfiguration("car_name")),

                launch_ros.actions.Node(
                    package='rktl_control',
                    executable='particle_odom_filter',
                    name='particle_odom_filter',
                    output='screen',
                    condition=IfCondition(
                        EqualsSubstitution(LaunchConfiguration('use_particle_filter'), 'true')
                    ),
                    parameters=[
                        {
                            PathJoinSubstitution([FindPackageShare('rktl_control'), '/config/particle_odom_filter.yaml'])
                        },
                        {
                            'frame_ids/body': LaunchConfiguration('car_name')
                        }
                    ]
                ),

                launch_ros.actions.Node(
                    package='rktl_control',
                    executable='mean_odom_filter',
                    name='mean_odom_filter',
                    output='screen',
                    condition=IfCondition(
                        EqualsSubstitution(LaunchConfiguration('use_particle_filter'), 'true')
                    ),
                    parameters=[
                        {
                            PathJoinSubstitution([FindPackageShare('rktl_control'), '/config/mean_odom_filter.yaml'])
                        },
                        {
                            'frame_ids/body': LaunchConfiguration('car_name')
                        }
                    ]
                ),

                launch_ros.actions.Node(
                    package='rktl_control',
                    executable='controller',
                    name='controller',
                    output='screen',
                    parameters=[
                        {
                            PathJoinSubstitution([FindPackageShare('rktl_control'), '/config/controller.yaml'])
                        }
                    ]
                )
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()