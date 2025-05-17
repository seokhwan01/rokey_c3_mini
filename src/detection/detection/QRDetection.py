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

    def target_room_callback(self, request, response):
        self.get_logger().info('Trying to connect to the camera...')
        while True:
            self.cap = cv2.VideoCapture(f'http://192.168.0.5:8080/video')
            if self.cap.isOpened():
                self.get_logger().info('Successfully connected to the camera.')
                break
            else:
                self.get_logger().error('Failed to connect to the camera.')
                time.sleep()

        ret, frame = self.cap.read()
        if ret:  
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)

            # QR 코드 감지 및 디코딩
            retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
            if retval:
                for info, pts in zip(decoded_info, points):
                    if info:
                        self.get_logger().info(f'QR 코드 감지됨: {info}')
                        pts = pts.astype(int)

                        # 사각형 그리기
                        for j in range(4):
                            pt1 = tuple(pts[j])
                            pt2 = tuple(pts[(j+1)%4])
                            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                        # 텍스트 출력
                        cv2.putText(frame, info, tuple(pts[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                        response.success = True
                        response.message = info
            else:
                response.success = False
                response.message = 'QR code not found'

        else:
            response.success = False
            response.message = 'Failed to capture image'
            self.get_logger().error('Failed to capture image')
        
        cv2.imshow('QR Detection', frame)
        cv2.waitKey(1)

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
