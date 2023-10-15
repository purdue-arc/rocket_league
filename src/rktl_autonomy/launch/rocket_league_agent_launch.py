from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plot_log',
            default_value='false',
            description='Set to true to enable logging for plotting performance.'
        ),
        DeclareLaunchArgument(
            'weights_dir',
            default_value='~/catkin_ws/data/rocket_league/',
            description='Directory for weights.'
        ),
        DeclareLaunchArgument(
            'weights_name',
            default_value='model',
            description='Name of the weights file.'
        ),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('plot_log')),
            msg="Enabling performance plotting..."
        ),

        Node(
            package='your_package_name',  # Replace with your package name
            executable='rocket_league_agent',  # Replace with your executable name
            name='rocket_league_agent',
            output='screen',
            parameters=[{'weights': LaunchConfiguration('weights_dir') + LaunchConfiguration('weights_name')}],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('plot_log')),
            package='your_package_name',  # Replace with your package name
            executable='plotter',  # Replace with your executable name
            name='plotter',
            output='screen',
            remappings=[('~log', 'rocket_league_agent/log')],
        )
    ])
