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

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header

current_dir = '/home/rokey/rokey_c3_mini/src/mini_project/model'
model_path = os.path.join(current_dir, 'real_final_best.pt')

class DetectionManager(Node):
    def __init__(self, model):
        super().__init__('detection_manager')

        self.model = model
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'capture_image',
            self.image_callback,
            10)

        self.detection_pub = self.create_publisher(
            Detection2DArray,
            'detection_result',
            10
        )

        self.classNames = model.names if hasattr(model, 'names') else ['Object']
        self.should_shutdown = False

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(img, stream=True)

        detection_array = Detection2DArray()
        detection_array.header = Header()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = 'camera_frame'

        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                detection = Detection2D()
                detection.bbox.center.position.x = (x1 + x2) / 2
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.classNames[cls]
                hypothesis.hypothesis.score = conf

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

                self.get_logger().info(
                    f"Detected {self.classNames[cls]} at ({x1}, {y1}, {x2}, {y2}) with {conf:.2f} confidence"
                )

        self.detection_pub.publish(detection_array)
        self.get_logger().info(f"Published {len(detection_array.detections)} detections to /detection_result")

def main():
    if not os.path.exists(model_path):
        print(f"❌ File not found: {model_path}")
        sys.exit(1)

    model = YOLO(model_path)

    rclpy.init()
    node = DetectionManager(model)
    print("yolo_detection 노드 실행")

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ Shutdown complete.")


if __name__ == '__main__':
    main()
