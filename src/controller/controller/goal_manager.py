import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import GetGoalPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import json

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator



class GoalManager(TurtleBot4Navigator):

    def __init__(self):
        super().__init__('goal_manager')

        # Service client to trigger QR detection (CameraNode)
        self.qr_service_client = self.create_client(Trigger, 'target_room')

        # Service client to get goal pose
        # self.goal_pose_client = self.create_client(GetGoalPose, 'get_goal_pose')

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
                self.request_goal_pose(qr_id)
            else:
                self.get_logger().error(f'QR detection failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def request_goal_pose(self, qr_id: str):
        json_path = ''
        with open(json_path, 'r') as f:
            goal_pose = json.load(f)
        
        goal_pose = navigator.getPoseStamped([goal_pose['x'], goal_pose['y']], -2.6)

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
