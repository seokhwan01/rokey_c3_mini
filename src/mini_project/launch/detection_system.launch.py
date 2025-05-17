#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('mini_project'),
            'param',
            'mini_project.yaml'))

    ros_namespace = LaunchConfiguration(
        'ros_namespace',
        default='/robot8'
        # default=os.getenv("ROS_NAMESPACE", "default_ns")
    )

    model_path = LaunchConfiguration(
        'model_path',
        default=os.path.join(
            get_package_share_directory('mini_project'),
            'model',
            'best.pt'))
    
    launch_description = LaunchDescription()

    param = DeclareLaunchArgument(
        'param_dir', 
        default_value=param_dir,
        description='Full path of detection parameter file')

    namespace = DeclareLaunchArgument(
        'ros_namespace',
        default_value=ros_namespace,
        description='Namespace for detection nodes')
    
    # Node
    image_publisher_node = Node(
        package='mini_project',
        executable='ImageConvertor',
        name='image_convertor',
        namespace=ros_namespace,
        parameters=[param_dir],
        # parameters=[param_dir, {'model_path': model_path}],
        output='screen')
    
    detection_manager_node = Node(
        package='mini_project',
        executable='DetectionManager',
        name='detection_manager',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
        
    tracking_manager_node = Node(
        package='mini_project',
        executable='TrackingManager',
        name='tracking_manager',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')

    display_manager_node = Node(
        package='mini_project',
        executable='DisplayManager',
        name='display_manager',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')

    qr_detection = Node(
        package='mini_project',
        executable='QRDetection',
        name='qr_detection',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
    
        
    launch_description.add_action(param)
    launch_description.add_action(namespace)
    launch_description.add_action(image_publisher_node)
    launch_description.add_action(detection_manager_node)
    launch_description.add_action(tracking_manager_node)
    launch_description.add_action(display_manager_node)
    launch_description.add_action(qr_detection)
    
    return launch_description
