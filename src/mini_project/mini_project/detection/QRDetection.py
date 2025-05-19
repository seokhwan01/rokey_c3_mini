import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import numpy as np

import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('ip_stream_node')

        self.service_ = self.create_service(Trigger, 'target_room', self.target_room_callback)

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.get_logger().info('Wait Request')
        self.last_qr_time = 0


    def target_room_callback(self, request, response):
        self.get_logger().info('Trying to connect to the camera...')
        ip_address = input('IP address: ')
        while True:
            self.cap = cv2.VideoCapture(f'http://{ip_address}:8080/video')
            if self.cap.isOpened():
                self.get_logger().info('Successfully connected to the camera.')
                break
            else:
                self.get_logger().error('Failed to connect to the camera.')
                time.sleep()

        while True:
            current_time = time.time()
            self.get_logger().info(f'current time: {current_time:.2f}')
            if current_time - self.last_qr_time >= 1.0:  # 1초 지났으면 처리
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().warn('Failed to capture frame')
                    continue            
                frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)            
                retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
                if retval and decoded_info:
                    decoded_info = decoded_info[0]

                    if decoded_info in ['1', '2', '3']:
                        self.get_logger().info('QR code found')
                        self.get_logger().info(f'detection: {True}')
                        self.get_logger().info(f'QR Id: {decoded_info}')
                        response.success = True
                        response.message = decoded_info
                        self.last_qr_time = current_time  # 시간 업데이트
                        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
