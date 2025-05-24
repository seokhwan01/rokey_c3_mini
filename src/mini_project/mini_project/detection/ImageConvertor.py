import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class ImageConvertor(Node):
    def __init__(self):
        super().__init__('image_convertor')

        # 'camera_image'라는 토픽에 sensor_msgs.msg.Image 타입의 메시지를 보내는 퍼블리셔.
        self.subscription = self.create_subscription(
            CompressedImage,
            'oakd/rgb/image_raw/compressed',
            self.listener_callback,
            10)
        
        self.publisher_ = self.create_publisher(Image, 'capture_image', 10)
        #imageConvertor node
        #'/capture_image'로 수정 필요해 보임
        # /붙으몉 절대 경로 안붙으면 상대경로
        # 슬래쉬 안붙으면 <namespace>/capture_image로 나감, 네임스페이스 지정안하고 실행시켜서 
        #상관은 없을 것 같지만 나중에 할때는 절대 경로 설정 필요해 보임
        # '/capture_image' 면 네임스페이스 지정해도 토픽 이름 변화 X
        # 로봇 두대로 할경우 '/robot8/image_capture' , '/robot7/image_capture'로 지정해줄 필요성 있음

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
    image_convertor = ImageConvertor()

    try:
        rclpy.spin(image_convertor)
    except KeyboardInterrupt:
        pass
    finally:
        image_convertor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
