from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    plot_log_launch_arg = DeclareLaunchArgument(
        'plot_log',
        default_value='true',
        description='Set to true to enable logging for plotting performance.'
        )
    
    agent_name_launch_arg = DeclareLaunchArgument(
        'agent_name',
        default_value='rocket_league_agent',
        description='Name of the agent.'
        )
    
    log_file_launch_arg = DeclareLaunchArgument(
        'log_file',
        default_value='rocket_league_agent/log',
        description='Filepath for logger output'
        )
    
    render_launch_arg = DeclareLaunchArgument(
            'render',
            default_value='false',
            description='Set to true to enable rendering.'
        )
    
    sim_mode_launch_arg = DeclareLaunchArgument(
            'sim_mode',
            default_value='ideal',
            description='Simulation mode.'
        )

    rate_launch_arg = DeclareLaunchArgument(
        'rate',
        default_value='10.0',
        description='Rate parameter.'
        )
    
    agent_type_launch_arg = DeclareLaunchArgument(
        'agent_type',
        default_value='none',
        description='Agent type.'
        )
    
    plot_log_info = LogInfo(
        condition=IfCondition(LaunchConfiguration('plot_log')),
        msg="Enabling performance plotting..."
        )
    
    agent_node = Node(
        package='rktl_autonomy',
        executable='rocket_league_agent',
        name=LaunchConfiguration('agent_name'),
        output='screen',
        parameters=[{'rate': LaunchConfiguration('rate')}],
        namespace=LaunchConfiguration('agent_name')
        )
    
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['src/rktl_launch/launch/rocket_league_sim.launch.py']),  # TODO: Replace with the path to the launch file
        launch_arguments={
            'render': LaunchConfiguration('render'),
            'sim_mode': LaunchConfiguration('sim_mode'),
            'agent_type': LaunchConfiguration('agent_type'),
        }.items(),
        )
    
    plotter_node = Node(
        condition=IfCondition(LaunchConfiguration('plot_log')),
        package='rktl_autonomy',
        executable='plotter',
        name='plotter',
        output='screen',
        remappings=[('~log', LaunchConfiguration('log_file'))],
        namespace=LaunchConfiguration('agent_name')
        )
    
    return LaunchDescription([
        plot_log_launch_arg,
        agent_name_launch_arg,
        log_file_launch_arg,
        render_launch_arg,
        sim_mode_launch_arg,
        rate_launch_arg,
        agent_type_launch_arg,
        plot_log_info,
        agent_node,
        sim_launch,
        plotter_node
    ])
