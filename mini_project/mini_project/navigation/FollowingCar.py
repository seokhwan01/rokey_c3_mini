import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO
import threading
import time
import cv2
import math
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator,TurtleBot4Directions

class FollowingCar(Node):
    def __init__(self):
        super().__init__('following_car')

        # === 초기 도킹 및 Pose 설정 ===
        self.navigator = TurtleBot4Navigator()

        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        self.get_logger().info('Init Pose')
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        
        self.get_logger().info('waitUntilNav2Active')
        self.navigator.waitUntilNav2Active()
        
        self.get_logger().info('Undock')
        self.navigator.undock()
        
        # Internal state
        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.latest_map_point = None

        self.goal_handle = None
        self.shutdown_requested = False
        self.logged_intrinsics = False
        self.current_distance = None
        self.close_enough_distance = 0.5  # meters
        self.block_goal_updates = False
        self.close_distance_hit_count = 0  # To avoid reacting to a single bad reading

        # # Load YOLOv8 model
        # self.model = YOLO("/home/rokey/to_students/day3/yolov8n.pt")
        self.model = YOLO("/home/rokey/rokey_c3_mini/src/mini_project/model/real_final_best.pt")

        # ROS 2 TF and Nav2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, '/robot8/navigate_to_pose')

        # Display
        self.display_frame = None
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        # ROS 2 subscriptions
        self.create_subscription(CameraInfo, '/robot8/oakd/rgb/preview/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/robot8/oakd/rgb/preview/image_raw', self.rgb_callback, 10)
        self.create_subscription(CompressedImage, '/robot8/oakd/stereo/image_raw/compressedDepth', self.compressed_depth_callback, 10)
        # self.create_subscription(Detection2DArray, '/detection_result', self.detection_callback, 10)

        # Periodic detection and goal logic
        self.create_timer(0.5, self.process_frame)
        self.last_feedback_log_time = 0

        # ★ 5초 후에 변환 시작 타이머 설정
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

        self.offset = 0.7
        
    def start_transform(self):
        # 첫 변환 시도
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        if hasattr(self, 'start_timer'):
            self.start_timer.cancel()
        
    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                                   f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def compressed_depth_callback(self, msg: CompressedImage):
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
            self.get_logger().error(f"CompressedDepth conversion failed: {e}")

    def process_frame(self):
        if self.K is None or self.rgb_image is None or self.depth_image is None:
            return

        results = self.model(self.rgb_image, verbose=False)[0]
        frame = self.rgb_image.copy()

        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            # Draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if label.lower() == "car":
                u = int((x1 + x2) // 2)
                v = int((y1 + y2) // 2)

                # z = (float(self.depth_image[v, u]) - 500) / 1000
                
                depth_crop = self.depth_image[v-1:v+2, u-1:u+2]
                z = (np.median(depth_crop) - 500) / 1000.0

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

                pt = PointStamped()
                pt.header.frame_id = self.camera_frame
                pt.header.stamp = rclpy.time.Time().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z

                try:
                    pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    self.latest_map_point = pt_map
            
                    # Don't send more goals if we're already close
                    if self.block_goal_updates:
                        self.get_logger().info(f"Within ({self.close_enough_distance}) meter — skipping further goal updates.")
                        break

                    self.get_logger().info(f"Detected person at map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")

                    if self.goal_handle:
                        self.get_logger().info("Canceling previous goal...")
                        self.goal_handle.cancel_goal_async()

                    self.send_goal()

                except Exception as e:
                    self.get_logger().warn(f"TF transform to map failed: {e}")
                break

        self.display_frame = frame

    def send_goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_map_point.point.x
        pose.pose.position.y = self.latest_map_point.point.y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        # Require 3 close readings to trigger the lock
        if self.current_distance is not None and self.current_distance < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
            self.block_goal_updates = True
            self.get_logger().info("Confirmed: within 1 meter — blocking further goal updates.")

        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {self.current_distance:.2f} m")
            self.last_feedback_log_time = now

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.goal_handle = None

    def display_loop(self):
        while rclpy.ok():
            if self.display_frame is not None:
                cv2.imshow("YOLO Detection", self.display_frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
                elif key == ord('r'):
                    self.block_goal_updates = False
                    self.close_distance_hit_count = 0
                    self.get_logger().info("Manual reset: goal updates re-enabled.")
            time.sleep(0.01)


def main():
    rclpy.init()
    node = FollowingCar()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

