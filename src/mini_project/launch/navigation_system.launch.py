from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


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
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of detection parameter file'),

        DeclareLaunchArgument(
            'ros_namespace',
            default_value=ros_namespace,
            description='Namespace for detection nodes'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('mini_project'),
                    'launch',
                    'setting_system.launch.py'
                )
            )
        ),

        GroupAction([
            PushRosNamespace(ros_namespace),

            Node(
                package='mini_project',
                executable='FollowingCar',
                name='following_car',
                parameters=[param_dir],
                output='screen'),

            Node(
                package='mini_project',
                executable='GoalPoseExtractor',
                name='goalPoseExtractor',
                parameters=[param_dir],
                output='screen'),

            Node(
                package='mini_project',
                executable='NavToPose',
                name='nav_to_pose',
                parameters=[param_dir],
                output='screen'),

            Node(
                package='mini_project',
                executable='GoalManager',
                name='goal_manager',
                parameters=[param_dir],
                output='screen'),
        ])
    ])
