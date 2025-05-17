import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 'camera_image'라는 토픽에 sensor_msgs.msg.Image 타입의 메시지를 보내는 퍼블리셔.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot8/oakd/rgb/image_raw/compressed',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'capture_image', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # Decode compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            ros_image.header = msg.header  # Preserve timestamp, frame_id, etc.

            # Publish the decoded image
            self.publisher_.publish(ros_image)
            self.get_logger().info('Image published successfully.')
        except Exception as e:
            self.get_logger().error(f'image decoding fail: {e}')
        
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
