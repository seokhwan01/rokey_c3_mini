import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import cv2
import json
import time 


class GoalPoseExtractor(Node):
    def __init__(self):
        super().__init__('goal_pose_extractor')

        self.bridge = CvBridge()
        self.K = None
        self.rgb_topic = '/capture_image'
        self.depth_topic = '/robot8/oakd/stereo/image_raw/compressedDepth'
        self.info_topic = '/robot8/oakd/rgb/preview/camera_info'

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_image = None
        
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(CompressedImage, self.depth_topic, self.depth_callback, 10)
        self.create_subscription(Detection2DArray, '/detection_result', self.detection_callback, 10)
        self.create_timer(0.1, self.display_images)
        
        self.logged_intrinsics = False

        self.points = []  # 저장할 포인트 리스트
        self.best_detections = {
            "1": {"confidence": 0.0, "data": None},
            "2": {"confidence": 0.0, "data": None},
            "3": {"confidence": 0.0, "data": None}
        }

        self.offset = 0.3
        self.detections = []
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

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert capture image: {e}")
        self.get_logger().info("✅ Successed to convert capture image")

    def depth_callback(self, msg):
        if self.rgb_image is None:
            self.get_logger().warn("Waiting for RGB image...")
            return
        if self.K is None:
            self.get_logger().warn("Camera intrinsics not received yet...")
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)

            png_start = np_arr.tobytes().find(b'\x89PNG')
            if png_start == -1:
                self.get_logger().error("CompressedDepth data does not contain PNG signature.")
                return

            depth_image = cv2.imdecode(np_arr[png_start:], cv2.IMREAD_UNCHANGED)
            if depth_image is None:
                self.get_logger().error("cv2.imdecode failed: returned None")
                return

            self.depth_image = depth_image

        except Exception as e:
            self.get_logger().error(f"Depth image conversion failed: {e}")
            return
        
        for detection in self.detections:
            if not detection.results:
                continue

            best_result = max(detection.results, key=lambda r: r.hypothesis.score)
            label = self.class_names_rev.get(best_result.hypothesis.class_id)
            conf = best_result.hypothesis.score

            bbox = detection.bbox
            cx = int(bbox.center.position.x)
            cy = int(bbox.center.position.y)
            x1 = max(cx - int(bbox.size_x // 2), 0)
            y1 = max(cy - int(bbox.size_y // 2), 0)
            x2 = min(cx + int(bbox.size_x // 2), self.depth_image.shape[1])
            y2 = min(cy + int(bbox.size_y // 2), self.depth_image.shape[0])

            if label.lower() in ["1", "2", "3"]:
                u = int((x1 + x2) // 2)
                v = int((y1 + y2) // 2)

                z = float(self.depth_image[v, u]) / 1000
                self.get_logger().info(f"Pixel ({u}, {v}), Depth: {z:.2f}m")
  
                if z == 0.0:
                    self.get_logger().warn("Depth value is 0 at detected number's center.")
                    continue

                if z < 0.2 or z > 3.0:
                    self.get_logger().warn(f"Filtered out: depth z out of range (z={z:.2f})")
                    continue
                
                if v < 0 or v >= self.depth_image.shape[0] or u < 0 or u >= self.depth_image.shape[1]:
                    self.get_logger().warn("Depth image index out of bounds")
                    continue

                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                # Auto-detect the camera frame from the depth image header
                camera_frame = msg.header.frame_id
                self.get_logger().info(f"camera_frame_id ({camera_frame})")
                self.get_logger().info(f"camera_frame: ({x:.2f}, {y:.2f}, {z:.2f})")

                # Prepare PointStamped in camera frame
                pt = PointStamped()
                pt.header.frame_id = camera_frame
                pt.header.stamp = msg.header.stamp  # for base_link we can use this timestamp
                pt.point.x = x
                pt.point.y = y
                pt.point.z = z

                try:
                    # Update header with latest time to avoid extrapolation error
                    pt_latest = PointStamped()
                    pt_latest.header.frame_id = camera_frame
                    pt_latest.header.stamp = rclpy.time.Time().to_msg()
                    pt_latest.point = pt.point

                    pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    self.get_logger().info(f"map:          ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")

                    # save goal_poses
                    self.handle_detection(label, conf, pt_map)

                except Exception as e:
                    self.get_logger().warn(f"TF to map failed: {e}")
        
    def detection_callback(self, msg: Detection2DArray):
        self.detections = msg.detections

    def handle_detection(self, label, current_confidence, pt_map):
            label = label.lower()
            x, y, z = pt_map.point.x, pt_map.point.y, pt_map.point.z
            
            theta = np.arctan2(y, x)  # 대략적인 방향 각도 계산 (단순 적용)
            x_offset = x - self.offset * np.cos(theta)
            y_offset = y - self.offset * np.sin(theta)

            # ✅ map 경계 필터링 (x:[0~3.6], y:[0~3.0])
            if not (-2.7 <= x <= 0.3 and -0.6 <= y <= 3.0):
                self.get_logger().warn(f"Filtered out: map coords out of bounds (x={x:.2f}, y={y:.2f})")
                return
    
            if label in self.best_detections:
                if current_confidence > self.best_detections[label]["confidence"]:
                    self.best_detections[label]["confidence"] = current_confidence
                    self.best_detections[label]["data"] = {
                        "label": label,
                        "x": x_offset,
                        "y": y_offset,
                        "z": z,
                        "confidence": current_confidence,
                        "timestamp": time.time()
                    }

                    # 라벨별로 JSON 저장
                    with open(f"/home/rokey/rokey3_C1_ws/map/best_{label}.json", 'w') as f:
                        self.get_logger().info(f"best_{label}.json is saved!")
                        json.dump(self.best_detections[label]["data"], f, indent=2)

    def display_images(self):
        if self.rgb_image is None:
            return

        rgb = self.rgb_image.copy()
        
        depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.applyColorMap(depth_vis.astype(np.uint8), cv2.COLORMAP_JET)

        if rgb.shape[:2] != depth_vis.shape[:2]:
            depth_vis = cv2.resize(depth_vis, (rgb.shape[1], rgb.shape[0]))

        stacked = np.hstack((rgb, depth_vis))
        cv2.imshow("RGB (left) | Depth (right)", stacked)

        key = cv2.waitKey(1)
        if key == 27:  # ESC 눌렀을 때 종료
            rclpy.shutdown()


def main():
    rclpy.init()
    node = GoalPoseExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# This script converts depth images to 3D point clouds and transforms them to the map frame.
# It uses the camera intrinsics and depth information to compute the 3D coordinates.
# The script also handles TF transformations to convert the points from the camera frame to the base_link and map frames.
