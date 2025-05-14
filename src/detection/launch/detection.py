#!/usr/bin/env python3
# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
            get_package_share_directory('hangman_game'),
            'param',
            'hangman_config.yaml'))

    ros_namespace = LaunchConfiguration(
        'ros_namespace',
        default=os.getenv("ROS_NAMESPACE", "default_ns")
    )
    
    launch_description = LaunchDescription()

    launch_description.add_action(DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file')
    )

    launch_description.add_action(DeclareLaunchArgument(
            'ros_namespace',
            default_value=ros_namespace,
            description='Namespace for the nodes'),
    )

    letter_publisher_node = Node(
        package='hangman_game',
        executable='letter_publisher',
        name='letter_publisher', 
        namespace=ros_namespace,
        parameters=[param_dir], 
        output='screen')

    word_service_node = Node(
        package='hangman_game',
        executable='word_service',
        name='word_service',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')

    progress_action_server_node = Node(
        package='hangman_game',
        executable='progress_action_server', 
        name='progress_action_server',
        namespace=ros_namespace, 
        parameters=[param_dir], 
        output='screen')

    progress_action_client_node = Node(
        package='hangman_game',
        executable='progress_action_client',
        name='progress_action_client',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')

    user_input_node = Node(
        package='hangman_game',
        executable='user_input',
        name='user_input',
        namespace=ros_namespace,
        parameters=[param_dir],
        output='screen')

    launch_description.add_action(letter_publisher_node)
    launch_description.add_action(word_service_node)
    launch_description.add_action(progress_action_server_node)
    launch_description.add_action(progress_action_client_node)
    launch_description.add_action(user_input_node)

    return launch_description