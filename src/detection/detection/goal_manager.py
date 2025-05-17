import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from example_interfaces.srv import GetGoalPose
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        # Subscribe to detected object ID
        self.subscription = self.create_subscription(
            String,
            '/object_id',
            self.object_callback, 10
        )

        self.goal_pose_client = self.create_client(GetGoalPose, 'get_goal_pose')
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


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()