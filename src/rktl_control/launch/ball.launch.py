import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            namespace='ball',
            package='rktl_control',
            executable='mean_odom_filter',
            name='mean_odom_filter',
            output='screen',
            parameters=[
                {
                    PathJoinSubstitution([FindPackageShare('rktl_control'), '/config/mean_odom_filter.yaml'])
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()