import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[
                {
                    'ground_height': '0.00'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[
                {
                    'ground_height': '-0.02'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[
                {
                    'ground_height': '-0.01'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[
                {
                    'ground_height': '-0.03'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz',
            executable='rviz',
            name='rviz'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
