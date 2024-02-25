import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='focus_vis',
            name='focus_vis'
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='focus_vis',
            name='focus_vis'
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='focus_vis',
            name='focus_vis'
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='focus_vis',
            name='focus_vis'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
