import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='car_name',
            default_value='car0'
        ),
        launch_ros.actions.Node(
            namespace=launch.substitutions.PathJoinSubstitution(['cars/', launch_ros.substitutions.FindPackageShare('car_name')]),
            package='rktl_control',
            executable='keyboard_interface',
            name='keyboard_interface',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()