#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # Launch arguments
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('mini_project'),
            'param',
            'mini_project.yaml'))

    ros_namespace = LaunchConfiguration(
        'ros_namespace',
        default='/robot8'
    )

    model_path = LaunchConfiguration(
        'model_path',
        default=os.path.join(
            get_package_share_directory('mini_project'),
            'model',
            'real_final_best.pt'))

    # Declare launch arguments
    param_arg = DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='Full path of detection parameter file')

    namespace_arg = DeclareLaunchArgument(
        'ros_namespace',
        default_value=ros_namespace,
        description='Namespace for detection nodes')

    model_arg = DeclareLaunchArgument(
        'model_path',
        default_value=model_path,
        description='Full path of YOLO model file')

    # Nodes
    image_publisher_node = Node(
        package='mini_project',
        executable='ImageConvertor',
        name='image_convertor',
        parameters=[param_dir],
        output='screen')

    detection_manager_node = Node(
        package='mini_project',
        executable='DetectionManager',
        name='detection_manager',
        parameters=[param_dir, {'model_path': model_path}],
        output='screen')

    tracking_manager_node = Node(
        package='mini_project',
        executable='TrackingManager',
        name='tracking_manager',
        parameters=[param_dir],
        output='screen')

    display_manager_node = Node(
        package='mini_project',
        executable='DisplayManager',
        name='display_manager',
        parameters=[param_dir],
        output='screen')

    qr_detection_node = Node(
        package='mini_project',
        executable='QRDetection',
        name='qr_detection',
        parameters=[param_dir],
        output='screen')

    # Group nodes under namespace
    grouped_nodes = GroupAction([
        PushRosNamespace(ros_namespace),
        image_publisher_node,
        detection_manager_node,
        tracking_manager_node,
        display_manager_node,
        qr_detection_node
    ])

    # Create launch description and add actions
    ld = LaunchDescription()

    ld.add_action(param_arg)
    ld.add_action(namespace_arg)
    ld.add_action(model_arg)
    ld.add_action(grouped_nodes)

    return ld
