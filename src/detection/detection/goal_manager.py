import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import GetGoalPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from your_custom_msgs.msg import QrInfo  # QR 메시지 타입


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        # Subscribe to detected object ID (String 형태)
        self.subscription = self.create_subscription(
            String,
            '/object_id',
            self.object_callback,
            10
        )

        # Subscribe to QR info (ID + Pose 포함)
        self.qr_subscription = self.create_subscription(
            QrInfo,
            '/qr_info',
            self.qr_callback,
            10
        )

        # Service client to get goal pose
        self.goal_pose_client = self.create_client(GetGoalPose, 'get_goal_pose')

        # Action client to send goal
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('GoalManager node has been started.')

    def object_callback(self, msg):
        object_id = msg.data
        self.get_logger().info(f'Received object ID: {object_id}')

        # Wait for service
        while not self.goal_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for get_goal_pose service...')

        # Create request
        request = GetGoalPose.Request()
        request.id = object_id

        # Async call and add callback
        future = self.goal_pose_client.call_async(request)
        future.add_done_callback(self.handle_goal_response)

    def handle_goal_response(self, future):
        try:
            response = future.result()
            self.get_logger().info('Received goal pose.')
            self.send_goal(response.pose)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_goal(self, pose: PoseStamped):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info('Sending goal to navigation...')
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
        # Optionally handle result.status here

    def qr_callback(self, msg: QrInfo):
        qr_id = msg.qr_id
        qr_pose = msg.pose  # geometry_msgs/Pose

        self.get_logger().info(f"[QR Info 수신] ID: {qr_id}, Pose: {qr_pose.position}")

        # QR 위치를 바로 목적지로 사용하려면 아래 주석 해제
        # pose_stamped = PoseStamped()
        # pose_stamped.header.frame_id = "map"
        # pose_stamped.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.pose = qr_pose
        # self.send_goal(pose_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
