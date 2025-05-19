#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()
    #turtlebot4_navigator 내부에서 ROS 2 Node가 생성되고 사용됩니다.
    
    ##여기서 위치 추정 또하냐???????
    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock() #

    # # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    # goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
    navigator.info('Setting Goal pose')
    # goal_pose = navigator.getPoseStamped([-2.32, -0.067], TurtleBot4Directions.EAST) # car
    
    goal_pose = navigator.getPoseStamped([-1.42, -0.595], TurtleBot4Directions.EAST) # "2"

    # goal_pose = navigator.getPoseStamped([-1.8, 0.008], -2.6)
#   Position(-1.55069, 0.0668084, 0), Orientation(0, 0, -0.962154, 0.272507) = Angle: -2.5896

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.info('Send Goal pose')
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
