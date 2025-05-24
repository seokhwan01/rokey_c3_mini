from enum import Enum
import time
import datetime
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool
from std_srvs.srv import Trigger, SetBool
from qr_interfaces.srv import GetTargetRoom # TODO: 커스텀 서비스로 변경 예정

from collections import deque
from mini_project.navigation.utils import create_pose, euclidean, load_goals_from_json # for ros2 run 
# from utils import create_pose, euclidean, load_goals_from_json # for python3 run

import socketio #소켓 추가~~~~~~~~~~~~~~~~~~~~~~~

# ------------------------
# Enum: 로봇 상태 정의
# ------------------------
class RobotStatus(Enum):
    IDLE = "idle"           # 대기 중 (작업 미할당)
    WAIT = "wait"           # 충돌 등으로 잠시 대기
    MOVING = "moving"       # 목표 지점으로 이동 중
    FINISHED = "finished"   # 배송 완료됨
    RETURNING = "returning" # 초기 위치 복귀 중

# ------------------------
# 로봇 상태 머신 클래스 (로봇 1대당 1개 생성)
# ------------------------
class RobotStateMachine:
    def __init__(self, name, init_pose, publisher, logger, sio=None): # 소켓 추가sio=None추가
        self.name = name
        self.state = RobotStatus.IDLE  # 초기 상태
        self.init_pose = init_pose     # 초기 위치
        self.current_pose = init_pose.pose
        self.goal_pose = None
        self.goal_finished = False
        self.wait_start_time = None
        self.blocked_until = 0.0
        self.publisher = publisher
        self.logger = logger
                
        # DB
        self.box_id = None
        self.room_num = None
        self.delivery_start_time = None

    # 현재 위치 갱신 (Odometry 콜백에서 호출)
    def update_pose(self, pose):
        self.current_pose = pose

    # 작업 완료 여부 갱신 (task_done 콜백에서 호출)
    def set_goal_finished(self, finished):
        self.goal_finished = finished

    # 현재 위치와 주어진 pose 사이 거리 계산
    def distance_to(self, pose):
        return euclidean(self.current_pose, pose)

    # 초기 위치 도달 여부 판단
    def is_at_init(self, threshold=0.3):
        return self.distance_to(self.init_pose.pose) < threshold

    # goal 할당 시 호출 (goal_pose: 배송 목적지 또는 복귀 목적지)
    def assign_goal(self, goal_pose, box_id=None, room_num=None):
        self.goal_pose = goal_pose
        self.box_id = box_id
        self.room_num = room_num
        self.delivery_start_time = time.time()
        self.state = RobotStatus.MOVING
        self._send_goal(goal_pose)
        self.logger.info(f"[{self.name}] goal 할당 → box_id={box_id}, room={room_num}")

    # 실제 goal 전송 메시지 발행
    def _send_goal(self, pose):
        msg = Float32MultiArray()
        msg.data = [pose.pose.position.x, pose.pose.position.y]
        self.publisher.publish(msg)

    # 충돌 시 goal 취소
    def cancel_goal(self):
        cancel_msg = Float32MultiArray()
        cancel_msg.data = [-1.0, -1.0]  # navigator가 무시하도록 설정
        self.publisher.publish(cancel_msg)
        self.logger.warn(f"[{self.name}] 작업 취소 → 상태: wait")
        self.wait_start_time = time.time()
        self.state = RobotStatus.WAIT

    # 상태 머신 주기적 업데이트 (1초마다 tick)
    def tick(self, now, is_too_close):
        if self.state == RobotStatus.IDLE:
            return  # 외부에서 goal 할당 대기

        # wait 상태: 5초 지나면 다시 이전 goal 재전송
        if self.state == RobotStatus.WAIT:
            if self.wait_start_time and time.time() - self.wait_start_time > 5.0:
                self.logger.info(f"[{self.name}] wait 종료 → goal 재전송")
                self._send_goal(self.goal_pose)
                self.state = RobotStatus.MOVING

        # 이동 중일 때
        elif self.state == RobotStatus.MOVING:
            if is_too_close:
                self.cancel_goal()
                self.blocked_until = now + 5.0
            elif self.goal_finished:
                self.goal_finished = False
                self.state = RobotStatus.FINISHED

                end_time = time.time()
                if self.delivery_start_time is not None:
                    elapsed = end_time - self.delivery_start_time
                else:
                    elapsed = -1

                start_str = datetime.datetime.fromtimestamp(self.delivery_start_time).strftime("%Y-%m-%d %H:%M:%S")
                end_str = datetime.datetime.fromtimestamp(end_time).strftime("%Y-%m-%d %H:%M:%S")

                log_data = { ###이름만 통합 내가 하는게 빠른가
                    "robot_name": self.name,
                    "box_id": self.box_id,
                    "room_num": self.room_num,
                    "start_time": start_str,
                    "end_time": end_str,
                }

                # 필드명 매핑 db랑 이름 매핑 소켓 추가
                send_log = {
                    "robot_name": log_data["robot_name"],
                    "parcel_id": log_data["box_id"],
                    "room_number": log_data["room_num"],
                    "received_time": log_data["start_time"],
                    "delivered_time": log_data["end_time"]
                }

                self.logger.info(f"[{self.name}] 📦 배송 완료! box_id={self.box_id} → {self.room_num} | 소요 시간: {elapsed:.2f}초")

                if self.sio and self.sio.connected: #웹서켓 연결 안되도 오류 안나게 소켓 추가
                    self.sio.emit("robot_delivery_log", send_log)
                    self.logger.info(f"[{self.name} send success]")

                # asyncio.create_task(send_delivery_record(log_data)) # 웹서버 전달용

                # 초기화
                self.delivery_start_time = None
                self.box_id = None
                self.room_num = None


        # 배송 완료 후 → 초기 위치로 복귀 명령
        elif self.state == RobotStatus.FINISHED:
            self.assign_goal(self.init_pose)
            self.state = RobotStatus.RETURNING

        # 초기 위치 복귀 중
        elif self.state == RobotStatus.RETURNING:
            if is_too_close:
                self.cancel_goal()
                self.blocked_until = now + 5.0
            elif self.is_at_init():
                self.state = RobotStatus.IDLE
                self.goal_pose = None
                self.logger.info(f"[{self.name}] 초기 위치 도착 → 상태: idle")

# ------------------------
# TaskManager Node 본체
# ------------------------
class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')
        ###소켓 추가~``
        self.sio = socketio.Client()
        try:
            self.sio.connect('http://192.168.0.8:5000')  # 필요시 IP나 포트 수정 
            #내 컴 터틀봇 8연결 ip 주소 192.168.0.8 
            self.get_logger().info('🌐 웹소켓 연결 성공')
        except Exception as e:
            self.get_logger().error(f'❌ 웹소켓 연결 실패: {e}')
            self.sio = None  # 연결 실패 시 None으로 설정
        ##추가~~~~
        
        self.robot_list = ["robot8", "robot9"]
        self.goal_queue = deque()  # 형식: (box_id: str, room_num: str, goal_pose: PoseStamped)
        self.robots = {}

        # Following Car Service 용
        self.start_flag = True  # ✅ 최초 1회 작업 트리거
        # self.stop_client = self.create_client(SetBool, '/stop_following', callback_group=ReentrantCallbackGroup())
        self.stop_clients = {
            "robot8": self.create_client(SetBool, '/robot8/stop_following', callback_group=ReentrantCallbackGroup()),
            # 나중에 robot9도 추가 가능
            }
        
        self.docking_signal = [-99.0, -99.0]  # 🚩 도킹 신호 (robotNavigator가 인식)
        self.docking_sent = {name: False for name in self.robot_list}

        # 로봇 FSM 등록
        for name in self.robot_list:
            pub = self.create_publisher(Float32MultiArray, f"navigator_interface", 10)
            init_x = 0.0 if name == "robot8" else 1.0  # 초기 위치 각자 할당
            pose = create_pose(init_x, 0.0, self.get_clock())
            # self.robots[name] = RobotStateMachine(name, pose, pub, self.get_logger())
            self.robots[name] = RobotStateMachine(name, pose, pub, self.get_logger(), sio=self.sio)# 소켓 추가이렇게 수정 소켓넘겨줌

            # 위치 및 완료 콜백 연결
            self.create_subscription(Odometry, f"odom", lambda msg, n=name: self.robots[n].update_pose(msg.pose.pose), 10)
            self.create_subscription(Bool, f"task_done", lambda msg, n=name: self.robots[n].set_goal_finished(msg.data), 10)

        # 서비스 요청 → goal 추가
        # self.create_service(Trigger, "/target_room", self.assign_task_callback)
        # TODO: 커스텀 서비스로 변경 예정
        self.create_service(GetTargetRoom, "target_room", self.assign_task_callback)

        # 주기적으로 FSM 상태 갱신
        self.create_timer(1.0, self.tick)

    # 서비스 콜백 → JSON에서 goal 로드 후 큐에 추가
    def assign_task_callback(self, request, response):
        self.call_stop_service("robot8")  # QR 인식 -> 팔로우 중지 서비스 요청

        # box_num, room_num = '1', '1'
        box_num, room_num = request.qr_id, request.target_room  # TODO: 커스텀 서비스로 변경 예정
        file_path = f"/home/rokey/rokey3_C1_ws/map/best_{room_num}.json"
        data = load_goals_from_json(file_path)

        if not data:
            response.success = False
            response.message = "[INFO] JSON file not found."
            return response

        pose = create_pose(data["x"], data["y"], self.get_clock())
        self.goal_queue.append((box_num, room_num, pose))
        response.success = True
        response.message = f"[INFO] Goal assigned for box {box_num}"
        return response
    
    # Following Car Service 중단 요청 ------------------------------
    def call_stop_service(self, robot_name):
        client = self.stop_clients.get(robot_name)
        if client is None:
            self.get_logger().warn(f"❌ {robot_name}에 대한 stop_following client가 없습니다.")
            return

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'🚨 /{robot_name}/stop_following 서비스 대기 중...')

        request = SetBool.Request()
        request.data = True

        future = client.call_async(request)
        future.add_done_callback(lambda fut: self.stop_response_callback(fut, robot_name))
        
    def stop_response_callback(self, future, robot_name):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f'✅ [{robot_name}] Following 중단 요청 성공: {res.message}')
            else:
                self.get_logger().warn(f'⚠️ [{robot_name}] Following 중단 실패: {res.message}')
        except Exception as e:
            self.get_logger().error(f'❌ [{robot_name}] 서비스 호출 예외 발생: {e}')
    # --------------------------------------------------------------

    # 1초마다 FSM 상태 업데이트 + goal 할당
    def tick(self):
        now = time.time()

        # 로봇 간 거리 계산 (충돌 방지)
        close = euclidean(self.robots["robot8"].current_pose, self.robots["robot9"].current_pose) < 1.2

        for name, robot in self.robots.items():
            # FSM 상태 갱신
            robot.tick(now, is_too_close=close)

            # idle 상태일 때 goal이 남아 있다면 할당
            if robot.state == RobotStatus.IDLE and self.goal_queue:
                box_id, room_num, goal_pose = self.goal_queue.popleft()
                robot.assign_goal(goal_pose, box_id=box_id, room_num=room_num)

        # 모든 로봇이 IDLE이고 작업 큐가 비었을 때 → 도킹 신호
        if not self.goal_queue:
            for name, robot in self.robots.items():
                if robot.state == RobotStatus.IDLE and not self.docking_sent[name]:
                    self.send_docking_signal(name)
                    self.docking_sent[name] = True  # 중복 방지

    # Docking Signal 발행 ------------------------------
    def send_docking_signal(self, robot_name):
        msg = Float32MultiArray()
        msg.data = self.docking_signal
        self.robots[robot_name].publisher.publish(msg)
        self.get_logger().info(f"📡 [{robot_name}] 도킹 신호 전송됨 (data={msg.data})")

# ---------------------- main 함수 ----------------------

def main():
    rclpy.init()
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()