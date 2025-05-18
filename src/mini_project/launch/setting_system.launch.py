#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('mini_project'),
            'param',
            'mini_project.yaml'))

    ros_namespace = LaunchConfiguration(
        'ros_namespace',
        default='/robot8')

    map_path = os.path.join(
        get_package_share_directory('your_package'),
        'map',
        'map.yaml')

    # lifecycle_manager_localization 노드 직접 선언
    lifecycle_manager_localization_node = Node(
        package='turtlebot4_navigation',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace=ros_namespace,
        output='screen',
        parameters=[param_dir]  # 필요한 파라미터 파일이나 딕셔너리 넣기
    )

    # localization launch 포함 (네임스페이스 적용)
    localization = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_navigation'),
                '/launch/localization.launch.py'
            ]),
            launch_arguments={'map': map_path}.items()
        )
    ])

    # navigation launch 포함 (네임스페이스 적용)
    nv = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_navigation'),
                '/launch/nav2.launch.py'
            ])
        )
    ])

    # visualization launch 포함 (네임스페이스 적용)
    visualization = GroupAction([
        PushRosNamespace(ros_namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot4_viz'),
                '/launch/naview_robotv2.launch.py'
            ])
        )
    ])

    # nav2 실행 트리거 - lifecycle_manager_localization 시작 시 실행
    nav2_trigger = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_manager_localization_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[nv]
                )
            ]
        )
    )

    # visualization 실행 트리거 - lifecycle_manager_navigation 시작 시 실행
    lifecycle_manager_navigation_node = Node(
        package='turtlebot4_navigation',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=ros_namespace,
        output='screen',
        parameters=[param_dir]
    )

    visualization_trigger = RegisterEventHandler(
        OnProcessStart(
            target_action=lifecycle_manager_navigation_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[visualization]
                )
            ]
        )
    )

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'param_dir', default_value=param_dir, description='Param file'))
    ld.add_action(DeclareLaunchArgument(
        'ros_namespace', default_value=ros_namespace, description='Namespace'))

    # 노드 및 그룹 실행 추가
    ld.add_action(localization)

    # lifecycle_manager 노드 직접 실행 (필요하다면)
    ld.add_action(lifecycle_manager_localization_node)
    ld.add_action(lifecycle_manager_navigation_node)

    # 이벤트 핸들러 추가
    ld.add_action(nav2_trigger)
    ld.add_action(visualization_trigger)

    return ld
