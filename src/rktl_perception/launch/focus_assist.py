import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []
    for i in range(4):
        node = Node(
            package='rktl_perception',
            executable='focus_vis',
            namespace=f'cams/cam{i}',
            name='focus_vis'
        )
        nodes.append(node)

    return LaunchDescription(nodes)

