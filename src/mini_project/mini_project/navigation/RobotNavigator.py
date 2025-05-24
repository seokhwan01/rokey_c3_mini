import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
from action_msgs.msg import GoalStatus
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions

class RobotNavigatorNode(Node):
    def __init__(self):
        super().__init__('robot_navigator_node')  # 노드 이름 설정

        # === TurtleBot4Navigator 초기화 ===
        self.navigator = TurtleBot4Navigator()

        if not self.navigator.getDockedStatus():
            self.get_logger().info('도킹 중...')
            self.navigator.dock()

        self.get_logger().info('초기 pose 설정 중...')
        ns = self.get_namespace()
        self.get_logger().info(f"현재 네임스페이스: {ns}")

        if ns == "/robot9":
            initial_pose = self.navigator.getPoseStamped([-0.01, 1.0], TurtleBot4Directions.NORTH) # robot9
            pre_pose = self.navigator.getPoseStamped([-0.5, 1.0], TurtleBot4Directions.SOUTH) # robot9
        elif ns == "/robot8":
            initial_pose = self.navigator.getPoseStamped([0.0,0.0], TurtleBot4Directions.NORTH) # robot8
            pre_pose = self.navigator.getPoseStamped([-0.5, 0.0], TurtleBot4Directions.SOUTH) # robot8
            
        self.navigator.setInitialPose(initial_pose)

        self.get_logger().info('Nav2 활성화 대기 중...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('언도킹 중...')
        self.navigator.undock()

        self.navigator.startToPose(pre_pose)

        # Nav2 액션 서버에 연결하기 위한 액션 클라이언트 생성
        # 이 클라이언트는 목표 좌표를 Nav2에 전송하고 결과를 기다리는 역할
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 좌표를 받을 토픽 구독 설정
        # Float32MultiArray 형태로 [x, y]를 받음
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'navigator_interface',  # ex. /robot8/navigator_interface, /robot9/navigator_interface
            self.listener_callback,
            10  # QoS 설정 (Queue size 10)
        )

        self.task_done_pub = self.create_publisher(Bool, 'task_done', 10)

        self.create_timer(1.0, self.check_task_result)
        self.result_future = None
        self.goal_handle = None

        self.task_done_sent = False  # ✅ 기존처럼 task 완료 신호 중복 방지
        self.docking_in_progress = False  # ✅ 도킹 수행 중인지 추적

        self.get_logger().info("Navigator Node ready. Listening to 'navigator_interface' topic.")

    def listener_callback(self, msg: Float32MultiArray):
        # 좌표 배열의 길이가 부족하면 무시
        if len(msg.data) < 2:
            self.get_logger().warn("Received data with less than 2 elements. Skipping.")
            return

        x, y = msg.data[0], msg.data[1]  # 받은 좌표값 추출
        
        # ✅ 도킹 신호 수신 시
        if x == -99.0 and y == -99.0:
            if not self.navigator.getDockedStatus():
                if not self.docking_in_progress:
                    self.get_logger().info("🛰 도킹 신호 수신 → 도킹 시도 중...")
                    self.navigator.dock()
                    self.docking_in_progress = True
                    self.task_done_sent = False  # ✅ 다음 goal을 위해 초기화
            else:
                self.get_logger().info("✅ 이미 도킹된 상태입니다.")
            return

        # 📌 취소 명령이면 현재 goal cancel
        if x == -1.0 and y == -1.0:
            if self.goal_handle is not None:
                self.get_logger().warn("[Navigator] Cancel command received. Cancelling current goal.")
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.cancel_result_callback)
            else:
                self.get_logger().warn("[Navigator] Cancel command received, but no active goal.")
            return
    
        self.task_done_sent = False  # ✅ 이전 완료 상태 초기화

        # 좌표를 포함한 PoseStamped 메시지 생성
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # 좌표계 기준 (map 좌표계 사용)
        pose.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 설정
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0  # 기본 방향 (회전 없음)

        # 액션 목표 설정
        goal = NavigateToPose.Goal()
        goal.pose = pose

        # Nav2 액션 서버가 준비될 때까지 대기
        self.get_logger().info(f"[Navigator] Sending goal to: ({x:.2f}, {y:.2f})")
        self.action_client.wait_for_server()

        # 비동기로 goal 전송
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)  # 전송 후 콜백 연결

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"🚫 Goal response 처리 중 오류: {e}")
            return

        # Goal이 거부되었을 경우
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return

        self.get_logger().info("Goal accepted.")

        # 목표 수행 결과 비동기 대기
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # 목표 완료 후 상태 코드 출력
        # result = future.result().result
        self.get_logger().info(f"Goal completed with status: {future.result().status}")

    def cancel_result_callback(self, future):
        cancel_response = future.result()
        if cancel_response.return_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("✅ Goal successfully cancelled.")
        else:
            self.get_logger().warn(f"❌ Goal cancel failed with code: {cancel_response.return_code}")
        self.result_future = None
        self.goal_handle = None

    def check_task_result(self):
        if self.result_future is not None and self.result_future.done():
            result = self.result_future.result().result
            status = self.result_future.result().status
            self.get_logger().info(f"Goal completed with status: {status}")

            success = (status == GoalStatus.STATUS_SUCCEEDED)

            # ✅ 한 번만 전송하도록 방지
            if not self.task_done_sent:
                self.task_done_pub.publish(Bool(data=success))
                self.task_done_sent = True  # 🔐 이후 전송 방지

                if success:
                    self.get_logger().info("✅ Goal succeeded. Task done signal published.")
                else:
                    self.get_logger().warn("❌ Goal failed. Failure signal published.")

            # 상태 초기화
            self.result_future = None
            self.goal_handle = None


def main():
    rclpy.init()  # ROS2 통신 초기화
    node = RobotNavigatorNode()
    node.get_logger().info(f"노드 생성 오나료")
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Navigator node shutting down.')  # 종료 로그
        node.destroy_node()  # 노드 파괴
        rclpy.shutdown()  # ROS2 종료


if __name__ == '__main__':
    main()
