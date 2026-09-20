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

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ZoneManagerNode(Node):
    def __init__(self):
        super().__init__('zone_manager_node')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('robot_ids', ['pinky1', 'pinky2'])
        # 로봇이 crash 등으로 notify_exit를 못 보낼 경우를 대비한 강제 해제 시간(초)
        self.declare_parameter('max_hold_sec', 120.0)

        self.robot_ids = list(self.get_parameter('robot_ids').value)
        self.max_hold_sec = float(self.get_parameter('max_hold_sec').value)

        self._lock = threading.Lock()
        self.occupant = None            # 현재 zone을 점유중인 robot_id (None이면 비어있음)
        self.occupant_since = None      # 점유 시작 시각 (max_hold_sec 감시용)
        self.queue = []                 # [(robot_id, token), ...] 대기열 (FIFO)

        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        # 요청 / 허가 / 이탈 통보 / 상태 토픽 (모두 patrol_cmd/status와 같은 String 토픽)
        self.create_subscription(String, '/zone_manager/request_entry', self._on_request, qos)
        self.create_subscription(String, '/zone_manager/notify_exit', self._on_exit, qos)
        self.grant_pub = self.create_publisher(String, '/zone_manager/grant_entry', qos)
        self.status_pub = self.create_publisher(String, '/zone_manager/status', qos)

        # max_hold_sec 초과 점유 감시
        self.create_timer(1.0, self._check_timeout)

        self.get_logger().info(f'Zone manager 시작 (robots={self.robot_ids}, '
                                f'max_hold_sec={self.max_hold_sec})')

    # ---------------- 콜백 ----------------

    def _on_request(self, msg: String):
        try:
            robot_id, token = msg.data.split(':', 1)
        except ValueError:
            self.get_logger().warn(f'잘못된 요청 형식: {msg.data}')
            return

        with self._lock:
            if self.occupant is None:
                self._set_occupant(robot_id)
                self._grant(robot_id, token)
            elif self.occupant == robot_id:
                # 이미 점유중인 로봇이 재요청(메시지 유실 대비 재전송 등) -> 즉시 재허가
                self._grant(robot_id, token)
            else:
                for i, (r, _t) in enumerate(self.queue):
                    if r == robot_id:
                        self.queue[i] = (robot_id, token)
                        break
                else:
                    self.queue.append((robot_id, token))
                    self.get_logger().info(
                        f'[{robot_id}] 대기열 등록 (현재 점유: {self.occupant}, '
                        f'대기중: {[r for r, _ in self.queue]})')

    def _on_exit(self, msg: String):
        robot_id = msg.data.strip()
        with self._lock:
            if self.occupant != robot_id:
                # 이미 타임아웃 등으로 해제됐거나, 다른 로봇 차례인 경우 -> 무시
                self.get_logger().warn(
                    f'[{robot_id}] notify_exit 수신했지만 현재 점유자는 '
                    f'{self.occupant} 입니다 (무시)')
                return
            self.get_logger().info(f'[{robot_id}] zone 이탈 통보 -> 락 해제')
            self._release_and_advance()

    def _check_timeout(self):
        with self._lock:
            if self.occupant is None or self.occupant_since is None:
                return
            held = time.time() - self.occupant_since
            if held > self.max_hold_sec:
                self.get_logger().warn(
                    f'[{self.occupant}] {held:.0f}초 동안 notify_exit 없음 '
                    f'-> 강제 해제 (max_hold_sec={self.max_hold_sec})')
                self._release_and_advance()

    # ---------------- 유틸 ----------------

    def _set_occupant(self, robot_id):
        self.occupant = robot_id
        self.occupant_since = time.time()
        self._publish_status()

    def _release_and_advance(self):
        self.occupant = None
        self.occupant_since = None
        self._publish_status()
        if self.queue:
            next_id, next_token = self.queue.pop(0)
            self._set_occupant(next_id)
            self._grant(next_id, next_token)

    def _grant(self, robot_id, token):
        msg = String()
        msg.data = f'{robot_id}:{token}'
        self.grant_pub.publish(msg)
        self.get_logger().info(f'[{robot_id}] 진입 허가 (token={token})')

    def _publish_status(self):
        msg = String()
        msg.data = f'occupied_by:{self.occupant}' if self.occupant else 'free'
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ZoneManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
