import rclpy
from rclpy.node import Node
from qr_interfaces.srv import GetTargetRoom
import cv2

###키보드 누를때 cv창에 마우스 두고 실행 터미널 X
class QRClientNode(Node):
    def __init__(self):
        super().__init__('qr_webcam_client')
        self.cli = self.create_client(GetTargetRoom, '/robot8/target_room')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('📡 서비스 대기 중...')
        self.get_logger().info('✅ 서비스 클라이언트 준비 완료')

    def send_service_request(self, qr_id: str,target_room:str):
        request = GetTargetRoom.Request()
        request.qr_id = qr_id
        request.target_room = target_room
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            res = future.result()
            if res.success:
                print(f"🎯 서비스 응답: message = {res.message}")
            else:
                print(f"❌ 서비스 실패: {res.message}")
        else:
            self.get_logger().error('🛑 서비스 호출 실패')


def main():
    rclpy.init()
    node = QRClientNode()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ 웹캠을 열 수 없습니다.")
        return

    qr_detector = cv2.QRCodeDetector()
    detected_qrs = []

    cv2.namedWindow("📷 QR 인식", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("📷 QR 인식", 800, 600)
    cv2.moveWindow("📷 QR 인식", 100, 100)

    print("✅ 웹캠 QR 감지 시작... [C: 인식] [ESC: 종료]") 

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("⚠️ 프레임을 읽을 수 없습니다.")
            break

        frame = cv2.resize(frame, (640, 480))
        key = cv2.waitKey(30)

        if key == ord('c'):
            print("🔍 QR 인식 시도 중...")
            retval, decoded_info, points, _ = qr_detector.detectAndDecodeMulti(frame)
            if retval:
                for info, pts in zip(decoded_info, points):
                    if info:
                        pts = pts.astype(int)
                        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
                        cv2.putText(frame, info, (pts[0][0], pts[0][1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                        if info in detected_qrs:
                            print(f"⚠️ 중복 QR 감지됨: {info}")
                        else:
                            detected_qrs.append(info)
                            print(f"✅ 새 QR 감지됨: {info}")
                            if '_' in info:
                                qr_id,target_room= info.split('_')
                                node.send_service_request(qr_id,target_room)
                            else:
                                print("⚠️ QR 형식 오류: 언더스코어(_) 없음")
            else:
                print("👀 QR 코드 감지되지 않음.")

        if key == 27:  # ESC
            break

        cv2.imshow("📷 QR 인식", frame)

    cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    print("🛑 종료되었습니다.")


if __name__ == '__main__':
    main()
