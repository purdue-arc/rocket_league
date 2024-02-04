#original 
#      <node pkg="rktl_perception" type="ball_color" name="ball_color" ns="cams/cam0" output="screen"/>
#     <!-- <node pkg="rktl_perception" type="ball_color" name="ball_color" ns="cams/cam1" output="screen"/> -->
#     <!-- <node pkg="rktl_perception" type="ball_color" name="ball_color" ns="cams/cam2" output="screen"/> -->
#     <!-- <node pkg="rktl_perception" type="ball_color" name="ball_color" ns="cams/cam3" output="screen"/> -->

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rktl_perception',
            executable='ball_color',
            name='ball_color',
            namespace='cams/cam0',
            output='screen'
        ),
        Node(
            package='rktl_perception',
            executable='ball_color',
            name='ball_color',
            namespace='cams/cam1',
            output='screen'
        ),
        Node(
            package='rktl_perception',
            executable='ball_color',
            name='ball_color',
            namespace='cams/cam2',
            output='screen'
        ),
        Node(
            package='rktl_perception',
            executable='ball_color',
            name='ball_color',
            namespace='cams/cam3',
            output='screen'
        ),
        
    ])