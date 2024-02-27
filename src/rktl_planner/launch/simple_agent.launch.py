import launch
import launch_ros.actions

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
                launch_ros.actions.PushRosNamespace(["agents/", launch.substitutions.LaunchConfiguration("agent_name")]),
                launch_ros.actions.Node(
                    package="rktl_planner",
                    executable="path_follower",
                    name="path_follower",
                    output="screen",
                    parameters=[
                        {
                            launch.substitutions.PathJoinSubstitution([launch_ros.substitutions.FindPackageShare('rktl_planner'), '/config/path_follower.yaml'])
                        },
                        {
                            "car_name": launch.substitutions.LaunchConfiguration("car_name")
                        }
                    ]
                ),
                launch_ros.actions.Node(
                    package="rktl_planner",
                    executable="bezier_path_server",
                    name="bezier_path_server",
                    output="screen"
                ),
                launch_ros.actions.Node(
                    package="rktl_planner",
                    executable="path_planner",
                    name="path_planner",
                    output="screen",
                    parameters=[
                        {
                            launch.substitutions.PathJoinSubstitution([launch_ros.substitutions.FindPackageShare('rktl_planner'), '/config/path_planner.yaml'])
                        },
                        {
                            "car_name": launch.substitutions.LaunchConfiguration("car_name")
                        }
                    ]
                )
            ]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
