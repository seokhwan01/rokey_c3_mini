import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        # === TurtleBot4Navigator 초기화 ===
        self.navigator = TurtleBot4Navigator()

        if not self.navigator.getDockedStatus():
            self.get_logger().info('도킹 중...')
            self.navigator.dock()

        self.get_logger().info('초기 pose 설정 중...')
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info('Nav2 활성화 대기 중...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('언도킹 중...')
        self.navigator.undock()

        # === 서비스 클라이언트 생성 ===
        self.qr_service_client = self.create_client(Trigger, 'target_room')

        # === JSON 파일 경로 설정 ===
        self.goal_pose_dict = self.load_goals_from_json('/home/rokey/rokey_C1_ws/src/controller/map/qr_goals.json')

        # === QR 감지 요청 시작 ===
        self.get_logger().info("QR 감지 요청 중...")
        self.call_qr_service()

    def load_goals_from_json(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"JSON 파일이 존재하지 않음: {filename}")
            return {}

        with open(filename, 'r') as f:
            data = json.load(f)
            self.get_logger().info(f"목표 pose를 JSON에서 불러옴: {filename}")
            return data

    def call_qr_service(self):
        while not self.qr_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('QR 감지 서비스 대기 중...')

        request = Trigger.Request()
        future = self.qr_service_client.call_async(request)
        future.add_done_callback(self.qr_response_callback)

    def qr_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                qr_id = response.message
                self.get_logger().info(f'QR 코드 인식됨: {qr_id}')
                self.handle_goal_from_json(qr_id)
            else:
                self.get_logger().error(f'QR 인식 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'QR 서비스 호출 실패: {e}')

    def handle_goal_from_json(self, qr_id: str):
        if qr_id not in self.goal_pose_dict:
            self.get_logger().error(f'QR ID {qr_id}는 JSON에 없음.')
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

        self.get_logger().info('JSON에서 pose를 불러옴. 자율주행 시작.')
        self.navigator.startToPose(pose)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
