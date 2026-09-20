#!/usr/bin/env python3
"""
zone_gate_client.py  (v2)

zone_manager_node(v2, self-report 방식)에게
"들어가도 되나요?"(request_entry) / "나왔어요"(notify_exit)를
보고하는 로봇 쪽 헬퍼 클래스.

사용 예 (PinkyPatrolNode 안에서):
    self.gate = ZoneGateClient(self.navigator, robot_id='pinky1')
    ...
    # zone에 들어가는 waypoint로 출발하기 직전
    ok = self.gate.wait_for_entry(stop_check=lambda: self._stop_requested)
    if not ok:
        # stop 명령 등으로 중단된 경우
        ...

    # zone을 벗어나는 waypoint에 도착한 직후
    self.gate.notify_exit()
"""

import time
import uuid

import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ZoneGateClient:
    def __init__(self, node, robot_id, retry_period_sec=1.0):
        """
        node: rclpy Node (또는 BasicNavigator처럼 Node를 상속한 객체).
              patrol 코드에서 이미 goToPose 폴링 등에 쓰고 있는 navigator를
              그대로 넘기면 됩니다 (별도 spin 충돌 걱정 없음).
        robot_id: zone_manager_node의 robot_ids 파라미터와 동일한 문자열
        """
        self.node = node
        self.robot_id = robot_id
        self.retry_period = retry_period_sec
        self._granted_token = None

        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self.request_pub = node.create_publisher(
            String, '/zone_manager/request_entry', qos)
        self.exit_pub = node.create_publisher(
            String, '/zone_manager/notify_exit', qos)
        self.grant_sub = node.create_subscription(
            String, '/zone_manager/grant_entry', self._on_grant, qos)

    def _on_grant(self, msg: String):
        try:
            robot_id, token = msg.data.split(':', 1)
        except ValueError:
            return
        if robot_id == self.robot_id:
            self._granted_token = token

    def wait_for_entry(self, timeout_sec=None, stop_check=None):
        """
        허가가 날 때까지 블로킹.

        stop_check: 인자 없이 호출해서 True/False를 반환하는 콜백.
                    True를 반환하면 즉시 대기를 중단합니다.
                    (예: patrol 노드의 stop 명령 처리용)

        반환값: 허가받았으면 True, timeout이나 stop_check로 중단되면 False.
        """
        token = uuid.uuid4().hex[:8]
        self._granted_token = None
        start = time.time()

        while rclpy.ok():
            if stop_check is not None and stop_check():
                return False

            msg = String()
            msg.data = f'{self.robot_id}:{token}'
            self.request_pub.publish(msg)

            end_wait = time.time() + self.retry_period
            while time.time() < end_wait:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                if self._granted_token == token:
                    self.node.get_logger().info(
                        f'[{self.robot_id}] zone 진입 허가 받음')
                    return True
                if stop_check is not None and stop_check():
                    return False
                if timeout_sec is not None and (time.time() - start) > timeout_sec:
                    return False

        return False

    def notify_exit(self):
        """zone을 완전히 벗어났음을 zone_manager에 통보 (락 즉시 해제)."""
        msg = String()
        msg.data = self.robot_id
        self.exit_pub.publish(msg)
        self.node.get_logger().info(f'[{self.robot_id}] zone 이탈 통보 전송')
