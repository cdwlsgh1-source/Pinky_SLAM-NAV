#!/usr/bin/env python3
"""
zone_manager_node.py  (v2 - self-report 방식)

여러 대의 로봇이 Nav2로 주행하는 중, 지정된 구역(zone)에 동시에
두 대 이상이 들어가지 못하도록 상호 배제(mutual exclusion)를 제공하는
중앙 관리 노드.

[v1과의 차이]
v1은 이 노드가 각 로봇의 amcl_pose를 직접 구독해서 zone 이탈을 감지했습니다.
하지만 로봇마다 도메인이 분리되어 있고 patrol_cmd/patrol_status 같은
가벼운 String 토픽만 브릿지하는 구조(예: PinkyPatrolNode)에서는
amcl_pose까지 브릿지해야 해서 배보다 배꼽이 커집니다.

v2는 로봇(클라이언트)이 이미 알고 있는 자기 위치(Nav2 feedback)를 이용해
"나 zone 들어간다"(request_entry) / "나 zone 나왔다"(notify_exit)를
직접 보고하는 방식입니다. 필요한 토픽은 patrol_cmd/patrol_status와
똑같은 성격의 가벼운 String 토픽 3개뿐이라 기존 브릿지 설정에
그대로 추가하기만 하면 됩니다.

[안전장치]
로봇이 크래시하거나 notify_exit를 못 보내는 상황에 대비해,
한 로봇이 max_hold_sec 이상 점유를 유지하면 자동으로 락을 강제 해제합니다.
"""

import threading   # 여러 콜백이 동시에 occupant/queue를 건드리지 못하게 잠그는 용도
import time        # 점유 시작 시각 기록, 타임아웃 계산용

import rclpy
from rclpy.node import Node   # ROS2 노드의 기본 클래스 (이걸 상속받아야 노드가 됨)
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ZoneManagerNode(Node):
    def __init__(self):
        super().__init__('zone_manager_node')   # 부모(Node) 초기화 + 노드 이름 등록

        # ---------------- 파라미터 ----------------
        # 실행 시 --ros-args -p 로 값을 바꿀 수 있게 파라미터로 선언 (코드 수정/재빌드 불필요)
        self.declare_parameter('robot_ids', ['pinky1', 'pinky2'])
        # 로봇이 crash 등으로 notify_exit를 못 보낼 경우를 대비한 강제 해제 시간(초)
        self.declare_parameter('max_hold_sec', 120.0)

        self.robot_ids = list(self.get_parameter('robot_ids').value)      # 파라미터 값 읽어서 리스트로 저장
        self.max_hold_sec = float(self.get_parameter('max_hold_sec').value)

        # ---------------- 상태 변수 ----------------
        self._lock = threading.Lock()   # occupant/queue를 건드릴 땐 항상 이 락을 잡고 들어감
        self.occupant = None            # 현재 zone을 점유중인 robot_id (None이면 비어있음)
        self.occupant_since = None      # 점유 시작 시각 (max_hold_sec 감시용)
        self.queue = []                 # [(robot_id, token), ...] 대기열 (FIFO, 먼저 온 순서대로 통과)

        # RELIABLE: 요청/허가 메시지는 유실되면 안 되므로 재전송 보장 옵션 사용
        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        # 요청 / 허가 / 이탈 통보 / 상태 토픽 (모두 patrol_cmd/status와 같은 String 토픽)
        # 로봇 쪽 ZoneGateClient와 정확히 반대 방향: 로봇이 publish하는 건 여기서 subscribe
        self.create_subscription(String, '/zone_manager/request_entry', self._on_request, qos)
        self.create_subscription(String, '/zone_manager/notify_exit', self._on_exit, qos)
        self.grant_pub = self.create_publisher(String, '/zone_manager/grant_entry', qos)
        self.status_pub = self.create_publisher(String, '/zone_manager/status', qos)   # 로봇 쪽엔 없는 신규 토픽

        # max_hold_sec 초과 점유 감시 - 1초마다 자동으로 _check_timeout 호출
        self.create_timer(1.0, self._check_timeout)

        self.get_logger().info(f'Zone manager 시작 (robots={self.robot_ids}, '
                                f'max_hold_sec={self.max_hold_sec})')

    # ---------------- 콜백 ----------------

    def _on_request(self, msg: String):
        """로봇이 'robot_id:token' 형식으로 진입 요청을 보내면 호출됨."""
        try:
            robot_id, token = msg.data.split(':', 1)   # "pinky1:a1b2c3" -> robot_id, token으로 분리
        except ValueError:
            self.get_logger().warn(f'잘못된 요청 형식: {msg.data}')   # ':'가 없는 등 형식이 이상하면 무시
            return

        with self._lock:   # 이 블록 안은 한 번에 하나의 콜백만 실행되도록 보장
            if self.occupant is None:
                # 케이스 1: 비어있음 -> 바로 점유자로 등록하고 즉시 허가
                self._set_occupant(robot_id)
                self._grant(robot_id, token)
            elif self.occupant == robot_id:
                # 케이스 2: 이미 점유중인 로봇이 재요청(메시지 유실 대비 재전송 등) -> 즉시 재허가만
                self._grant(robot_id, token)
            else:
                # 케이스 3: 다른 로봇이 쓰는 중 -> 대기열에 등록
                for i, (r, _t) in enumerate(self.queue):
                    if r == robot_id:
                        self.queue[i] = (robot_id, token)   # 이미 대기열에 있으면 토큰만 최신으로 갱신
                        break
                else:
                    # break 없이 루프가 끝까지 돌았다 = 대기열에 없었다 -> 새로 추가
                    self.queue.append((robot_id, token))
                    self.get_logger().info(
                        f'[{robot_id}] 대기열 등록 (현재 점유: {self.occupant}, '
                        f'대기중: {[r for r, _ in self.queue]})')

    def _on_exit(self, msg: String):
        """로봇이 zone을 벗어났다고 통보하면 호출됨 (응답 없음, fire-and-forget)."""
        robot_id = msg.data.strip()   # 앞뒤 공백/개행 제거
        with self._lock:
            if self.occupant != robot_id:
                # 이미 타임아웃 등으로 해제됐거나, 다른 로봇 차례인 경우 -> 무시 (뒤늦게 도착한 메시지 대비)
                self.get_logger().warn(
                    f'[{robot_id}] notify_exit 수신했지만 현재 점유자는 '
                    f'{self.occupant} 입니다 (무시)')
                return
            self.get_logger().info(f'[{robot_id}] zone 이탈 통보 -> 락 해제')
            self._release_and_advance()   # 점유 해제 + 대기열에 다음 로봇 있으면 통과시킴

    def _check_timeout(self):
        """1초마다 자동 호출. 점유자가 너무 오래(max_hold_sec 초과) 안 나오면 강제로 쫓아냄."""
        with self._lock:
            if self.occupant is None or self.occupant_since is None:
                return   # 아무도 점유중이 아니면 검사할 필요 없음
            held = time.time() - self.occupant_since   # 현재까지 점유한 시간(초)
            if held > self.max_hold_sec:
                self.get_logger().warn(
                    f'[{self.occupant}] {held:.0f}초 동안 notify_exit 없음 '
                    f'-> 강제 해제 (max_hold_sec={self.max_hold_sec})')
                self._release_and_advance()

    # ---------------- 유틸 ----------------

    def _set_occupant(self, robot_id):
        """점유자를 등록하고 점유 시작 시각을 기록, 상태를 방송."""
        self.occupant = robot_id
        self.occupant_since = time.time()
        self._publish_status()

    def _release_and_advance(self):
        """점유를 해제하고, 대기열에 로봇이 있으면 맨 앞 로봇을 바로 통과시킴."""
        self.occupant = None
        self.occupant_since = None
        self._publish_status()
        if self.queue:
            next_id, next_token = self.queue.pop(0)   # 대기열 맨 앞(FIFO)에서 꺼냄
            self._set_occupant(next_id)
            self._grant(next_id, next_token)

    def _grant(self, robot_id, token):
        """'robot_id:token' 형식으로 허가 메시지를 발행 (ZoneGateClient가 이 형식을 파싱함)."""
        msg = String()
        msg.data = f'{robot_id}:{token}'
        self.grant_pub.publish(msg)
        self.get_logger().info(f'[{robot_id}] 진입 허가 (token={token})')

    def _publish_status(self):
        """현재 zone 상태('free' 또는 'occupied_by:로봇id')를 방송."""
        msg = String()
        msg.data = f'occupied_by:{self.occupant}' if self.occupant else 'free'
        self.status_pub.publish(msg)


def main():
    rclpy.init()                 # ROS2 시스템 초기화
    node = ZoneManagerNode()     # 노드 생성 (여기서 __init__ 실행됨)
    try:
        rclpy.spin(node)         # 여기서 블로킹 -> 구독/타이머 콜백을 계속 처리
    except KeyboardInterrupt:
        pass                     # Ctrl+C로 종료해도 에러 없이 넘어감
    finally:
        node.destroy_node()      # 에러가 나든 안 나든 항상 노드 정리
        rclpy.shutdown()


if __name__ == '__main__':   # 이 파일을 직접 실행했을 때만 main() 호출 (import될 땐 실행 안 됨)
    main()
