#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


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
    
    launch_description = LaunchDescription()
    
    # evn setting
    param = DeclareLaunchArgument(
        'param_dir', 
        default_value=param_dir,
        description='Full path of detection parameter file')

    namespace = DeclareLaunchArgument(
        'ros_namespace',
        default_value=ros_namespace,
        description='Namespace for detection nodes')
    
    # Ather Launch Import
    setting_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(),
             '/setting_system.launch.py']
        )
    )

    # Node setting
    following_car_node = Node(
        package='mini_project',
        executable='FollowingCar',
        name='following_car',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
    
    goal_pose_extractor_node = Node(
        package='mini_project',
        executable='GoalPoseExtractor',
        name='goalPoseExtractor',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
    
    nav_to_pose_node = Node(
        package='mini_project',
        executable='NavToPose',
        name='nav_to_pose',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
    
    goal_manager = Node(
        package='mini_project',
        executable='GoalManager',
        name='goal_manager',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')
    
        
    launch_description.add_action(param)
    launch_description.add_action(namespace)
    
    launch_description.add_action(setting_system)

    launch_description.add_action(following_car_node)
    launch_description.add_action(goal_manager)
    launch_description.add_action(goal_pose_extractor_node)
    launch_description.add_action(nav_to_pose_node)

    return launch_description

