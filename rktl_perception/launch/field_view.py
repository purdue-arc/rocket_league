import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rktl_perception_share_dir = launch.substitutions.LaunchConfiguration('rktl_perception_share_dir',
                                                                        default=get_package_share_directory('rktl_perception'))

    return LaunchDescription([
        Node(
            namespace='cams/cam0',
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[{'ground_height': 0.00}]
        ),
        Node(
            namespace='cams/cam1',
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[{'ground_height': -0.02}]
        ),
        Node(
            namespace='cams/cam2',
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[{'ground_height': -0.01}]
        ),
        Node(
            namespace='cams/cam3',
            package='rktl_perception',
            executable='projector',
            name='projector',
            parameters=[{'ground_height': -0.03}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rktl_perception_share_dir + '/rviz/field.rviz']
        )
    ])

