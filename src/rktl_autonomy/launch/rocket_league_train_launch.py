from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'plot_log',
            default_value='true',
            description='Set to true to enable logging for plotting performance.'
        ),
        DeclareLaunchArgument(
            'agent_name',
            default_value='rocket_league_agent',
            description='Name of the agent.'
        ),
        DeclareLaunchArgument(
            'log_file',
            default_value='rocket_league_agent/log',
            description='Filepath for logger output'),
        DeclareLaunchArgument(
            'render',
            default_value='false',
            description='Set to true to enable rendering.'
        ),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='ideal',
            description='Simulation mode.'
        ),

        DeclareLaunchArgument(
            'rate',
            default_value='10.0',
            description='Rate parameter.'
        ),
        DeclareLaunchArgument(
            'agent_type',
            default_value='none',
            description='Agent type.'
        ),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('plot_log')),
            msg="Enabling performance plotting..."
        ),

        Node(
            package='rktl_autonomy',
            executable='rocket_league_agent',
            name=LaunchConfiguration('agent_name'),
            output='screen',
            parameters=[{'rate': LaunchConfiguration('rate')}],
            namespace=LaunchConfiguration('agent_name')
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['$(find rktl_launch)/launch/rocket_league_sim_launch.py']),  # TODO: Replace with the path to the launch file
            launch_arguments={
                'render': LaunchConfiguration('render'),
                'sim_mode': LaunchConfiguration('sim_mode'),
                'agent_type': LaunchConfiguration('agent_type'),
            }.items(),
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('plot_log')),
            package='rktl_autonomy',
            executable='plotter',
            name='plotter',
            output='screen',
            remappings=[('~log', LaunchConfiguration('log_file'))],
            namespace=LaunchConfiguration('agent_name')
        )
    ])
