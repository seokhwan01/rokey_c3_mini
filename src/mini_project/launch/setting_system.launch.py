#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch.actions import TimerAction

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

    map_path = os.path.join(
        get_package_share_directory('your_package'),
        'map',
        'map.yaml'
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
    localization = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_navigation'),
                '/launch/localization.launch.py'
            ]),
            launch_arguments={
                'map': map_path
            }.items()
        )
    ])

    nv = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_navigation'),
                '/launch/nav2.launch.py'
            ])
        )
    ])

    visualization = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_viz'),
                '/launch/naview_robotv2.launch.py'
            ])
        )
    ])
    
    # Trigger Setting
    nav2_trigger = RegisterEventHandler(
        OnProcessStart(
            target_action=[ros_namespace, '/lifecycle_manager_localization'], # lifecycle_manager_localization 
            on_start=[
                TimerAction(
                    period=5.0, # period 후 nv 실행
                    actions=[nv]
                )
            ]
        )
    )
    
    visualization_trigger = RegisterEventHandler(
        OnProcessStart(
            target_action=[ros_namespace, '/lifecycle_manager_navigation'],
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[visualization]
                )
            ]
        )
    )
        
    launch_description.add_action(param)
    launch_description.add_action(namespace)
    
    launch_description.add_action(localization)
    # launch_description.add_action(
    #     TimerAction(
    #         period=60.0,  # period 후 nv 실행
    #         actions=[nv]
    #     )
    # )

    # launch_description.add_action(
    #     TimerAction(
    #         period=60.0,  # period 후 visualization 실행
    #         actions=[visualization]
    #     )
    # )
    launch_description.add_action(nav2_trigger)
    launch_description.add_action(visualization_trigger)

    return launch_description