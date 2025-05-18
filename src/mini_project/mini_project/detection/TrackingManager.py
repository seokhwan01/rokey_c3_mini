import time
import math
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import cv2
#추가
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

#받는거 capture_image
#발행 tracking_result 메시지 형태는 sudo apt install ros-humble-vision-msgs다운
#객체의 감지 정보(박스, 클래스, ID 등)를 구조화된 메시지로 퍼블리시

current_dir = '/home/rokey/rokey3_C1_ws/src/detection/detection'
model_path = os.path.join(current_dir, 'model/real_final_best.pt')

class TrackingManager(Node):
    def __init__(self, model):
        super().__init__('tracking_manager')
        self.model = model
        self.bridge = CvBridge() #  OpenCV 이미지와 ROS 이미지 메시지 사이를 변환하기 위한 **도구(브릿지)**를 만드는 코드

        #create_subscription()	터틀봇4의 RGB 이미지 토픽을 구독
        self.subscription = self.create_subscription(
            Image,
            '/capture_image', #받는 이미지 토픽 이름 , 나중에 설정에 따라 수정
            self.listener_callback,
            10)
        
         # 감지 결과 퍼블리셔 ('tracking_result') 추가
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/tracking_result',
            10
        )

        self.classNames = model.names if hasattr(model, 'names') else ['Object']
        self.should_shutdown = False

    def listener_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.track(source=img, stream=True, persist=True)

        ##발행을 위해 추가
        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_frame'
        ## 여기까지 추가


        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                track_id = int(box.id[0]) if box.id is not None else -1

                # 비전 메시지 구성 추가
                #감지된 객체 하나에 대해 vision_msgs/Detection2D 메시지를 구성하는 부분
                #이 정보를 보내는게 정확한지? 지피티 말대로 한거 다시 확인 할 수도
                detection = Detection2D() #vision_msgs/Detection2D 메시지 객체 생성

                detection.bbox.center.position.x = (x1 + x2) / 2
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)


                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.classNames[cls] + '_' + str(track_id) # ex: 'person'
                hypothesis.hypothesis.score = conf

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)
                ### 여기까지ㅡ추가

                # 로그 출력 디버깅용
                self.get_logger().info(
                    f"Detected {self.classNames[cls]} ID:{track_id} "
                    f"at ({x1}, {y1}, {x2}, {y2}) with {conf:.2f} confidence")

        #결과 퍼블리시       
        self.detection_pub.publish(detection_array)
        self.get_logger().info(f"Published detections to /tracking_result")

def main():
    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        sys.exit(1)

    model = YOLO(model_path)  # supports tracking with .track()

    rclpy.init()
    node = TrackingManager(model)
    print("yolo_track 노드 실행")
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
