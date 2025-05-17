import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        # Service client to trigger QR detection (CameraNode)
        self.qr_service_client = self.create_client(Trigger, 'target_room')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Start QR detection
        self.get_logger().info("Requesting QR detection...")
        self.call_qr_service()

    def call_qr_service(self):
        while not self.qr_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for target_room service...')

        request = Trigger.Request()
        future = self.qr_service_client.call_async(request)
        future.add_done_callback(self.qr_response_callback)

    def qr_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                qr_id = response.message
                self.get_logger().info(f'QR code detected: {qr_id}')

                # Load goal poses from JSON
                file_path = f'hoem/rokey/rokey_C1_ws/src/controller/map/best{qr_id}_go.json'
                self.goal_pose_dict = self.load_goals_from_json(file_path)

                self.handle_goal_from_json()
            else:
                self.get_logger().error(f'QR detection failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def load_goals_from_json(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"Goal JSON file not found: {filename}")
            return {}

        with open(filename, 'r') as f:
            data = json.load(f)
            self.get_logger().info(f"Loaded goal poses from {filename}")
            return data
        
    def handle_goal_from_json(self):

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = pose_data['position']['x']
        pose.pose.position.y = pose_data['position']['y']
        pose.pose.position.z = pose_data['position']['z']

        pose.pose.orientation.x = pose_data['orientation']['x']
        pose.pose.orientation.y = pose_data['orientation']['y']
        pose.pose.orientation.z = pose_data['orientation']['z']
        pose.pose.orientation.w = pose_data['orientation']['w']

        self.get_logger().info('Loaded goal pose from JSON.')
        self.send_goal(pose)

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Sending goal to navigation: {pose.pose.position}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result received: {result}')
        # result.status 등 추가 처리 가능


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        # Service client to trigger QR detection (CameraNode)
        self.qr_service_client = self.create_client(Trigger, 'target_room')

        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Load goal poses from JSON
        self.goal_pose_dict = self.load_goals_from_json('qr_goals.json')

        # Start QR detection
        self.get_logger().info("Requesting QR detection...")
        self.call_qr_service()

    def load_goals_from_json(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"Goal JSON file not found: {filename}")
            return {}

        with open(filename, 'r') as f:
            data = json.load(f)
            self.get_logger().info(f"Loaded goal poses from {filename}")
            return data

    def call_qr_service(self):
        while not self.qr_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for target_room service...')

        request = Trigger.Request()
        future = self.qr_service_client.call_async(request)
        future.add_done_callback(self.qr_response_callback)

    def qr_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                qr_id = response.message
                self.get_logger().info(f'QR code detected: {qr_id}')
                self.handle_goal_from_json(qr_id)
            else:
                self.get_logger().error(f'QR detection failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def handle_goal_from_json(self, qr_id: str):
        if qr_id not in self.goal_pose_dict:
            self.get_logger().error(f'QR ID {qr_id} not found in goal poses.')
            return

        pose_data = self.goal_pose_dict[qr_id]

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = pose_data['position']['x']
        pose.pose.position.y = pose_data['position']['y']
        pose.pose.position.z = pose_data['position']['z']

        pose.pose.orientation.x = pose_data['orientation']['x']
        pose.pose.orientation.y = pose_data['orientation']['y']
        pose.pose.orientation.z = pose_data['orientation']['z']
        pose.pose.orientation.w = pose_data['orientation']['w']

        self.get_logger().info('Loaded goal pose from JSON.')
        self.send_goal(pose)

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f'Sending goal to navigation: {pose.pose.position}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server.')
            return

        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result received: {result}')
        # result.status 등 추가 처리 가능


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
