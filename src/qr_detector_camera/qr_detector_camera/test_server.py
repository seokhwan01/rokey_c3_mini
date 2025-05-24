import rclpy
from rclpy.node import Node
from qr_interfaces.srv import GetTargetRoom


class TargetRoomServer(Node):
    def __init__(self):
        super().__init__('target_room_server')
        self.srv = self.create_service(GetTargetRoom, '/robot8/target_room', self.handle_request)
        self.get_logger().info('🚀 QR 서비스 서버 실행 중 (/robot8/target_room)')

        # 이미 등록된 데이터 확인용 (예시)
        self.known_pairs = set()

    def handle_request(self, request, response):
        qr_id = request.qr_id
        room = request.target_room

        log_msg = f"📥 요청 수신: qr_id = {qr_id}, target_room = {room}"
        self.get_logger().info(log_msg)

        pair = (qr_id, room)
        if pair in self.known_pairs:
            response.success = False
            response.message = f"⚠️ 이미 등록된 QR ID: {qr_id}"
        else:
            self.known_pairs.add(pair)
            response.success = True
            response.message = f"✅ QR {qr_id} → Room {room} 저장 완료"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetRoomServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("🛑 서버 종료됨.")


if __name__ == '__main__':
    main()
