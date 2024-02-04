# original code at bottom

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    load_manager = LaunchConfiguration('load_manager', default='true')
    manager_name = LaunchConfiguration('manager_name', default='camera_manager')
    manager_threads = LaunchConfiguration('manager_threads', default='4')
    camera_name = LaunchConfiguration('camera_name', default='cam0')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument('load_manager', default_value='true', description='Load the nodelet manager'),
        launch.actions.DeclareLaunchArgument('manager_name', default_value='camera_manager', description='Name for the nodelet manager'),
        launch.actions.DeclareLaunchArgument('manager_threads', default_value='4', description='Number of worker threads for the nodelet manager'),
        launch.actions.DeclareLaunchArgument('camera_name', default_value='cam0', description='Name for the camera'),

        launch.actions.Group(
            namespace='cams/' + camera_name,
            actions=[
                Node(
                    condition=launch.conditions.IfCondition(load_manager),
                    package='nodelet',
                    executable='nodelet',
                    name=manager_name,
                    arguments=['manager'],
                    parameters=[{'num_worker_threads': manager_threads}]
                ),
                Node(
                    package='nodelet',
                    executable='nodelet',
                    name=camera_name,
                    arguments=['load', 'pointgrey_camera_driver/PointGreyCameraNodelet', manager_name],
                    parameters=[{'frame_id': camera_name},
                                {'camera_info_url': 'package://rktl_perception/config/' + camera_name + '/calibration.yaml'}],
                    remappings=[('/camera/image_raw', '/image_raw'), ('/camera/camera_info', '/camera_info')]
                ),
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.XmlLaunchFileSource(
                        '/path/to/image_proc_launch_file'
                    ),
                    launch_arguments={'manager': manager_name}.items()
                ),
                Node(
                    package='apriltag_ros',
                    executable='apriltag_ros_continuous_node',
                    name='apriltag',
                    parameters=[{'camera_frame': camera_name, 'publish_tag_detections_image': True}],
                    remappings=[('/tag_detections_image', '/tag_detections_image')],
                ),
                Node(
                    package='rktl_perception',
                    executable='localizer',
                    name='localizer',
                    parameters=[{'origin_id': '0,1,2,3,4,5,6,7',
                                 'buffer_size': 300,
                                 'ball_sub_topic': 'ball_vec',
                                 'ball_pub_topic': '/ball/pose',
                                 'ball_radius': 0.03,
                                 'pub_topic': 'pose'}],
                ),
                Node(
                    package='rktl_perception',
                    executable='pose_to_tf',
                    name='pose_to_tf',
                    parameters=[{'cam_frame_id': camera_name}],
                ),
                Node(
                    package='rktl_perception',
                    executable='ball_detection',
                    name='ball_detection',
                )
            ]
        )
    ])

