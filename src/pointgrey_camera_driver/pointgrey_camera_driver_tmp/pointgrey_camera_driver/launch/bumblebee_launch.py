import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource

def generate_launch_description():
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value=TextSubstitution(text="camera")
    )
    bumlebee_serial_launch_arg = DeclareLaunchArgument(
        "bumblebee_serial", default_value=TextSubstitution(text="0")
    )
    calibrated_launch_arg = DeclareLaunchArgument(
        "calibrated", default_value=TextSubstitution(text="0")
    )

    node_group_namespace = GroupAction(
        actions=[
            PushRosNamespace(),
            bumblebee_nodelet = Node()
        ]
    )
