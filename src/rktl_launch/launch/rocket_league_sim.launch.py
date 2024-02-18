import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='render',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='sim_mode',
            default_value='realistic' # Either realistic or ideal (check docs)
        ),
        launch.actions.DeclareLaunchArgument(
            name='perception_delay',
            default_value='0.15'
        ),
        launch.actions.DeclareLaunchArgument(
            name='agent_type',
            default_value='patrol' # Either planner, autonomy, or patrol
        ),
        launch.actions.DeclareLaunchArgument(
            name='autonomy_weights',
            default_value='model'
        ),
        launch.actions.GroupAction(
            actions=[
                launch_ros.actions.PushRosNamespace("truth"),
                launch_ros.actions.SetRemap(
                    src="/cars/car0/odom",
                    dst="/cars/car0/odom_truth",
                ),
                launch_ros.actions.SetRemap(
                    src="/ball/odom",
                    dst="/ball/odom_truth",
                ),
                launch.actions.GroupAction(
                    actions=[
                        launch_ros.actions.PushRosNamespace("visualizer"),
                        launch_ros.actions.SetParameter(
                            name="window_name",
                            value="Ground Truth",
                        )
                    ],
                    condition=launch.conditions.LaunchConfigurationEquals('sim_mode', 'realistic')
                )
        
            ],
            condition=launch.conditions.LaunchConfigurationEquals('sim_mode', 'realistic')
        ),
        launch_ros.actions.SetParametersFromFile(
            filename=os.path.join(get_package_share_directory(
                    'rktl_launch'), 'config', 'global_params.yaml')
        ),
        launch_ros.actions.Node(
            namespace='ball',
            package='rktl_control',
            executable='topic_delay',
            name='pose_delay',
            condition=launch.conditions.LaunchConfigurationEquals('sim_mode', 'realistic'),
            arguments="pose_sync_early pose_sync geometry_msgs/PoseWithCovarianceStamped " + str(launch.substitutions.LaunchConfiguration('perception_delay'))
        ),
        launch_ros.actions.Node(
            namespace='cars/car0',
            package='rktl_control',
            executable='topic_delay',
            name='pose_delay',
            condition=launch.conditions.LaunchConfigurationEquals('sim_mode', 'realistic'),
            arguments="pose_sync_early pose_sync geometry_msgs/PoseWithCovarianceStamped " + str(launch.substitutions.LaunchConfiguration('perception_delay'))
        ),
        launch_ros.actions.Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            condition=launch.conditions.LaunchConfigurationEquals('render', 'true'),
            arguments='--perspective-file ' + os.path.join(get_package_share_directory(
                    'rktl_launch'), 'rqt','rktl.perspective')
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_sim'), 'launch', 'visualizer.launch.py')
            ),
            namespace='truth',
            condition=launch.conditions.LaunchConfigurationEquals('render', 'true')
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_sim'), 'launch/simulator.launch.py')
            ),
            launch_arguments={
                'sim_mode': launch.substitutions.LaunchConfiguration('sim_mode')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/simple_agent.launch.py')
            ),
            launch_arguments={
                'agent_name': 'agent0',
                'car_name': 'car0'
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_autonomy'), 'launch/rocket_league/rocket_league_agent.launch.py')
            ),
            launch_arguments={
                'weights_name': launch.substitutions.LaunchConfiguration('autonomy_weights')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'rktl_planner'), 'launch/patrol_agent.launch.py')
            ),
            launch_arguments={
                'car_name': 'car0'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
