#NOT FINISHED 10/18/2023
from ament_index_python import get_package_share_directory
import launch
from launch_ros.actions import Node, SetParametersFromFile
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    load_manager = DeclareLaunchArgument('load_manager', default_value='true')
    manager_name = DeclareLaunchArgument('manager_name', default_value='camera_manager')
    manager_threads = DeclareLaunchArgument('manager_threads', default_value='4')
    camera_name = DeclareLaunchArgument('camera_name', default_value='cam0')

    ns_group = launch.actions.GroupAction(
        actions=[
            Node(
                package='nodelet',
                executable='nodelet',
                name=[launch.substitutions.LaunchConfiguration('manager_name')],
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                arguments='manager',
                parameters=[{'num_worker_threads': launch.substitutions.LaunchConfiguration('manager_threads')}],
                condition=launch.conditions.IfCondition([launch.substitutions.LaunchConfiguration('load_manager')])
            ),
            Node(
                package='nodelet',
                executable='nodelet',
                name=[launch.substitutions.LaunchConfiguration('camera_name')],
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                arguments=['load', 'pointgrey_camera_driver/PointGreyCameraNodelet', launch.substitutions.LaunchConfiguration('manager_name')],
                parameters=[
                    {'frame_id': launch.substitutions.LaunchConfiguration('camera_name')},
                    {'camera_info_url': 'package://rktl_perception/config/' + launch.substitutions.LaunchConfiguration('camera_name') + '/calibration.yaml'}
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('image_proc'), 'launch', 'image_proc.launch')),
                launch_arguments={'manager': launch.substitutions.LaunchConfiguration('manager_name')}.items(),
            ),
            Node(
                package='apriltag_ros',
                executable='apriltag_ros_continuous_node',
                name='apriltag',
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                parameters=[
                    {'camera_frame': launch.substitutions.LaunchConfiguration('camera_name')},
                    {'publish_tag_detections_image': True},
                    os.path.join(get_package_share_directory('rktl_perception'), 'config', 'apriltag_settings.yaml'),
                    os.path.join(get_package_share_directory('rktl_perception'), 'config', 'tags.yaml')
                ]
            ),
            SetParametersFromFile(filename=os.path.join(get_package_share_directory('rktl_perception'), 'config', launch.substitutions.LaunchConfiguration('camera_name'), 'pointgrey.yaml')),

            Node(
                package='rktl_perception',
                executable='localizer',
                name='localizer',
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                parameters=[
                    {'origin_id': '0,1,2,3,4,5,6,7'},
                    {'buffer_size': 300},
                    {'ball_sub_topic': 'ball_vec'},
                    {'ball_pub_topic': '/ball/pose'},
                    {'ball_radius': 0.03},
                    {'pub_topic': 'pose'},
                    {'pub_topics': {
                        '0,1,2,3,4,5,6,7': '/origin/pose',
                        '10': '/cars/car0/pose'
                    }}
                ]
            ),
            Node(
                package='rktl_perception',
                executable='pose_to_tf',
                name='pose_to_tf',
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                parameters=[
                    {'cam_frame_id': launch.substitutions.LaunchConfiguration('camera_name')}
                ]
            ),
            Node(
                package='rktl_perception',
                executable='ball_detection',
                name='ball_detection',
                namespace=['cams/', launch.substitutions.LaunchConfiguration('camera_name')],
                parameters = os.path.join(get_package_share_directory('rktl_perception'), 'config', launch.substitutions.LaunchConfiguration('camera_name'), 'ball_settings.yaml')
            )
        ],
        namespace='cams/' + launch.substitutions.LaunchConfiguration('camera_name')
    )

    return launch.LaunchDescription([load_manager, manager_name, manager_threads, camera_name, ns_group])
