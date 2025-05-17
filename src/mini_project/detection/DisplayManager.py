import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis
from vision_msgs.msg import BoundingBox2D, Pose2D, Point2D
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class DisplayManager(Node):
    def __init__(self):
        super().__init__('display_manager')

        # 이미지 구독
        self.image_subscription = self.create_subscription(
            Image,
            '/capture_image',
            self.image_callback,
            10)
        
        # YOLO 탐지 결과 구독
        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detection_result',
            self.detection_callback,
            10)

        # # 추적 결과 구독
        self.tracking_subscription = self.create_subscription(
            Detection2DArray,
            '/tracking_result',
            self.tracking_callback,
            10)
        
        self.bridge = CvBridge()
        
        self.latest_frame = None
        
        # 클래스 ID → 이름 매핑
        self.class_names = {
            0: "0", 1: "1", 2: "2", 3: "3", 4: "4",
            5: "5", 6: "6", 7: "7", 8: "8", 9: "9",
            10: "Car", 11: "Person"
        }

        # 이름 매핑 → 클래스 ID 
        self.class_names_rev = {
            "0": 0, "1": 1, "2": 2, "3": 3, "4": 4,
            "5": 5, "6": 6, "7": 7, "8": 8, "9": 9,
            "Car": 10, "Person": 11 
        }
        
        # 객체 추적 정보 (ID와 바운딩 박스를 기록)
        self.tracking_info = {}
    
    def image_callback(self, msg):
        # 수신된 이미지 변환
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info("새로운 이미지 수신 완료")
    
    def detection_callback(self, msg: Detection2DArray):
        if self.latest_frame is None:
            self.get_logger().warn("이미지가 아직 수신되지 않았습니다.")
            return
        
        # 최신 프레임 복사
        image = self.latest_frame.copy()
        self.get_logger().info("탐지 결과 처리 중...")
        
        # 탐지된 객체를 바운딩 박스로 그리기
        for detection in msg.detections:
            if not detection.results:
                continue
            
            # 가장 신뢰도 높은 결과만 사용
            best_result = max(detection.results, key=lambda r: r.hypothesis.score)
            class_id = self.class_names_rev.get(best_result.hypothesis.class_id)  # int(best_result.hypothesis.class_id)
            score = best_result.hypothesis.score
            
            bbox = detection.bbox
            cx = bbox.center.position.x
            cy = bbox.center.position.y
            w = bbox.size_x
            h = bbox.size_y
            
            # 바운딩 박스 좌표 계산
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)
            label = self.class_names.get(class_id, f"class_{class_id}")
            color = (0, 0, 255)  # 빨간색 바운딩 박스
            
            # 바운딩 박스 그리기
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                image,
                f"{label}: {score:.2f}",
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

        # 시각화된 이미지 출력
        cv2.imshow("Detection Visualization", image)
        cv2.waitKey(1)

    def tracking_callback(self, msg: Detection2DArray):
        if self.latest_frame is None:
            self.get_logger().warn("이미지가 아직 수신되지 않았습니다.")
            return
        # 최신 프레임 복사
        image = self.latest_frame.copy()
        self.get_logger().info("추적 결과 처리 중...")
        # 추적된 객체를 바운딩 박스로 그리기
        for detection in msg.detections:
            if not detection.results:
                continue

            # 가장 신뢰도 높은 결과만 사용
            best_result = max(detection.results, key=lambda r: r.hypothesis.score)
            class_id = self.class_names_rev.get(best_result.hypothesis.class_id.split('_')[1])
            track_id = int(best_result.hypothesis.class_id.split('_')[1])  # 객체 ID 관리 (추적 정보)
            score = best_result.hypothesis.score

            bbox = detection.bbox
            cx = bbox.center.position.x
            cy = bbox.center.position.y
            w = bbox.size_x
            h = bbox.size_y

            # 바운딩 박스 좌표 계산
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)
            
            if track_id not in self.tracking_info:
                self.tracking_info[track_id] = (x1, y1, x2, y2)

            # 객체 ID에 대해 이전의 위치와 비교하여 추적
            previous_bbox = self.tracking_info[track_id]
            
            cv2.putText(
                image,
                f"{best_result.hypothesis.class_id}, ID {track_id}: {score:.2f}",
                (x1, max(y1 - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),  # 추적 객체는 초록색
                2
            )

            # 추적된 객체에 바운딩 박스 그리기
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # 초록색 바운딩 박스

        # 시각화된 이미지 출력
        cv2.imshow("Tracking Visualization", image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # OpenCV 창 닫기
        print("Subscriber shutdown complete.")
if __name__ == '__main__':
    main()






