import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


class LaneFollowerNode(Node):
    """
    lane_detector_node가 발행하는 <namespace>/lane/center_offset, <namespace>/lane/detected
    를 구독해 <namespace>/cmd_vel(geometry_msgs/Twist)을 발행하는 주행 제어 노드.

    제어 로직:
      - 최근(offset_timeout_sec 이내)에 lane/detected=True 였다면:
            linear.x  = linear_speed (고정 속도)
            angular.z = -kp * offset  (offset 부호에 비례하는 단순 P 제어)
      - 차선을 못 찾았거나(lane/detected=False) 일정 시간 이상 새 데이터가 없으면:
            즉시 정지 (linear.x = 0, angular.z = 0)

    주의: offset은 lane_detector_node 기준 '왼쪽=음수, 오른쪽=양수'로 정의되어 있음.
          ROS REP-103 기준 angular.z는 양수가 좌회전(반시계) 방향이므로,
          차선이 오른쪽에 있을 때(offset 양수) 오른쪽으로 꺾어야 하니
          angular.z는 음수가 되어야 함 -> angular_z = -kp * offset.
          실제 로봇에서 반대로 움직이면 kp 부호만 뒤집으면 됩니다.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        self.declare_parameter('kp', 1.5)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('offset_timeout_sec', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)

        self.kp = self.get_parameter('kp').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.offset_timeout_sec = self.get_parameter('offset_timeout_sec').value
        control_rate_hz = self.get_parameter('control_rate_hz').value

        # 카메라/인식 파이프라인과 동일하게 BEST_EFFORT + depth=1
        # (오래된 offset 값을 큐에서 처리하느라 지연되는 것을 방지)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._latest_offset = 0.0
        self._latest_detected = False
        self._last_update_time = None  # rclpy Time, 마지막으로 메시지를 받은 시각

        self.offset_sub = self.create_subscription(
            Float32, 'lane/center_offset', self._offset_callback, qos)
        self.detected_sub = self.create_subscription(
            Bool, 'lane/detected', self._detected_callback, qos)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 제어 루프는 인식 노드의 발행 주기에 휘둘리지 않도록 별도 고정 주기 타이머로 실행
        self.control_timer = self.create_timer(1.0 / control_rate_hz, self._control_loop)

        self._stopped_logged = False

        self.get_logger().info(
            f'lane_follower_node 시작: kp={self.kp}, linear_speed={self.linear_speed}, '
            f'max_angular_speed={self.max_angular_speed}, '
            f'offset_timeout_sec={self.offset_timeout_sec}, control_rate={control_rate_hz}Hz')

    def _offset_callback(self, msg: Float32):
        self._latest_offset = msg.data
        self._last_update_time = self.get_clock().now()

    def _detected_callback(self, msg: Bool):
        self._latest_detected = msg.data
        # detected 콜백도 '데이터가 살아있다'는 신호이므로 타임스탬프 갱신
        self._last_update_time = self.get_clock().now()

    def _control_loop(self):
        twist = Twist()

        timed_out = (
            self._last_update_time is None
            or (self.get_clock().now() - self._last_update_time).nanoseconds
            > self.offset_timeout_sec * 1e9
        )

        if timed_out or not self._latest_detected:
            # 안전 정지: 차선 정보가 없거나(lane/detected=False) 데이터가 오래된 경우
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if not self._stopped_logged:
                reason = '데이터 타임아웃' if timed_out else '차선 미검출'
                self.get_logger().warn(f'정지: {reason}')
                self._stopped_logged = True
        else:
            angular_z = -self.kp * self._latest_offset
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            twist.linear.x = self.linear_speed
            twist.angular.z = angular_z
            self._stopped_logged = False

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LaneFollowerNode()
    except Exception as e:
        print(f'[lane_follower_node] 노드를 시작할 수 없습니다: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 로봇이 마지막 명령으로 계속 움직이지 않도록 정지 명령을 한 번 발행
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()