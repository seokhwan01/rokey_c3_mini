import json
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
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


        # GR 인식
        # === 서비스 클라이언트 생성 ===
        self.qr_service_client = self.create_client(Trigger, 'target_room')
        self.follow_service_client = self.create_client(SetBool, 'stop_following')
        self.get_logger().info('follwing_car client ready')

        # === QR 감지 요청 시작 ===
        self.get_logger().info("QR 감지 요청 중...")
        self.call_qr_service()

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
                self.qr_id = response.message
                self.get_logger().info(f'QR 코드 인식됨: {self.qr_id}')

                self.get_logger().info(f'Following Car STOP!!')
                self.call_follow_service()

            else:
                self.get_logger().error(f'QR 인식 실패: {response.message}')
        except Exception as e:
            self.get_logger().error(f'QR 서비스 호출 실패: {e}')

    def call_follow_service(self):
        while not self.follow_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('follow 서비스 대기 중...')

        request = SetBool.Request()
        request = True
        future = self.qr_service_client.call_async(request)
        future.add_done_callback(self.follow_response_callback)

    def follow_response_callback(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f'Following state: {response.message}')

            # === JSON 파일 경로 설정 ===
            file_path = f'/home/rokey/rokey_c3_mini/src/mini_project/map/best_{self.qr_id}.json'
            goal_pose_dict = self.load_goals_from_json(file_path)

            self.handle_goal_from_json(goal_pose_dict)
        else:
            self.get_logger().info(f'Following state: {response.message}')

    def load_goals_from_json(self, filename):
        if not os.path.exists(filename):
            self.get_logger().error(f"JSON 파일이 존재하지 않음: \n{filename}")
            return {}

        with open(filename, 'r') as f:
            data = json.load(f)
            self.get_logger().info(f"목표 pose를 JSON에서 불러옴: \n{filename}")
            return data
        
    def handle_goal_from_json(self, goal_pose_dict):
        goal_pose = self.navigator.getPoseStamped([goal_pose_dict['x'], goal_pose_dict['y']], -2.6)

        self.get_logger().info('JSON에서 pose를 불러옴. 자율주행 시작.')
        self.navigator.startToPose(goal_pose)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()