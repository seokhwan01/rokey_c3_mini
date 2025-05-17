import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

import time


class CameraNode(Node):
    def __init__(self):
        super().__init__('ip_stream_node')

        self.service_ = self.create_service(Trigger, 'target_room', self.target_room_callback)

        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        self.get_logger().info('Wait service')


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
            ret, frame = self.cap.read()
            if ret:  
                frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)

                # QR 코드 감지 및 디코딩
                retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
                if retval:
                    if decoded_info:
                        decoded_info = decoded_info[0]
                        self.get_logger().info(f'QR 코드 감지됨: {decoded_info}')
                        pts = points.astype(int)
                        self.get_logger().info(str(pts))

                        # 사각형 그리기
                        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                        # 텍스트 출력
                        cv2.putText(frame, decoded_info, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

                    
                    if decoded_info in ['1', '2', '3']:
                        self.get_logger().info('QR code found')
                        self.get_logger().info(f'detection: {True}')
                        self.get_logger().info(f'QR Id: {decoded_info}')
                        response.success = True
                        response.message = decoded_info
                        return response

                else:
                    self.get_logger().error('QR code not found')

            else:
                self.get_logger().error('Failed to capture image')
            
            cv2.imshow('QR Detection', frame)
            cv2.waitKey(1)


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
