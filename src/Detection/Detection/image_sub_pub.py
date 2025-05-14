import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageRelayNode(Node):
    def __init__(self):
        super().__init__('image_relay_node')
        self.bridge = CvBridge()

        # RGB 이미지 구독 및 퍼블리시
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb_image',
            self.rgb_callback, 10
        )
        self.rgb_pub = self.create_publisher(
            Image,
            '/rgb_image_relay', 10
        )

        # Depth 이미지 구독 및 퍼블리시
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_image',
            self.depth_callback, 10
        )
        self.depth_pub = self.create_publisher(
            Image,
            '/depth_image_relay',10
        )

        self.get_logger().info('✅ Image Relay Node (RGB + Depth) started.')

    def rgb_callback(self, msg):
        # RGB 이미지 그대로 재전송
        self.rgb_pub.publish(msg)
        self.get_logger().info('🔁 Relayed RGB image.')

    def depth_callback(self, msg):
        # Depth 이미지 그대로 재전송
        self.depth_pub.publish(msg)
        self.get_logger().info('🔁 Relayed Depth image.')

def main(args=None):
    rclpy.init(args=args)
    node = ImageRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
