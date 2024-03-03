from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    plot_log = LaunchConfiguration('plot_log')  # True if plotting is enabled. Else False
    weights = LaunchConfiguration('weights')  # Filepath to weights

    plot_log_launch_arg = DeclareLaunchArgument(
        'plot_log',
        default_value='false',
        description='Set to true to enable logging for plotting performance.'
        )
    
    weights_launch_arg = DeclareLaunchArgument(
        'weights',
        default_value='~/catkin_ws/data/rocket_league/model',
        description='filepath to weights.'
        )
    
    plot_log_info = LogInfo(condition=IfCondition(plot_log), msg="Enabling performance plotting...")

    agent_node = Node(
        package='rktl_autonomy', 
        executable='rocket_league_agent',
        name='rocket_league_agent',
        output='screen',
        parameters=[{'weights': weights}],
        )
    
    plotter_node = Node(
        condition=IfCondition(plot_log),
        package='rktl_autonomy',
        executable='plotter', 
        name='plotter',
        output='screen',
        remappings=[('~log', 'rocket_league_agent/log')],
        )
    
    return LaunchDescription([
        plot_log_launch_arg, 
        weights_launch_arg,
        plot_log_info,
        agent_node,
        plotter_node
        ])
