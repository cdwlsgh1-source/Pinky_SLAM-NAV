import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class LaneFollowerNode(Node):
    """
    lane_detector_node가 발행하는 lane/center_offset, lane/detected(, crossline/detected)를
    구독해서 실제로 cmd_vel(Twist)을 publish하는 주행 노드.

    제어 방식: offset(-1.0 ~ 1.0, 0이 정중앙)에 대한 P(비례) + D(미분) 제어.
      angular.z = -(kp * offset + kd * d_offset/dt)
      (offset이 양수 = 차선 중심이 화면 오른쪽에 있음 -> 오른쪽으로 돌아야 하므로
       ROS 표준(양의 각속도 = 반시계/왼쪽 회전) 기준 angular.z는 음수 방향)

    안전장치:
      - 차선을 잃어버리면(lane_detected=False) lost_count를 누적하고,
        max_lost_frames를 넘기면 정지(linear.x=0)한다.
      - offset 토픽 자체가 일정 시간(watchdog_timeout) 이상 끊기면(카메라/추론 노드 다운)
        무조건 정지한다.
    """

    def __init__(self):
        super().__init__('lane_follower_node')

        self.declare_parameter('linear_speed', 0.15)          # 기본 직진 속도 (m/s)
        self.declare_parameter('min_linear_speed', 0.05)      # 많이 꺾을 때 최저 속도
        self.declare_parameter('kp', 1.2)                     # 비례 게인
        self.declare_parameter('kd', 0.3)                     # 미분 게인
        self.declare_parameter('max_angular_speed', 1.5)      # 각속도 제한 (rad/s)
        self.declare_parameter('max_lost_frames', 10)         # 차선 미검출 허용 프레임 수
        self.declare_parameter('watchdog_timeout', 0.5)       # offset 미수신 시 정지까지 시간(s)
        self.declare_parameter('stop_on_crossline', False)    # Crossline 검출 시 정지할지 여부
        self.declare_parameter('crossline_stop_duration', 2.0)  # 정지 유지 시간(s)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.stop_on_crossline = self.get_parameter('stop_on_crossline').value
        self.crossline_stop_duration = self.get_parameter('crossline_stop_duration').value

        self._prev_offset = 0.0
        self._prev_time = self.get_clock().now()
        self._lost_count = 0
        self._last_msg_time = self.get_clock().now()
        self._crossline_stop_until = None  # rclpy Time or None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(Float32, 'lane/center_offset', self.offset_callback, qos)
        self.create_subscription(Bool, 'lane/detected', self.detected_callback, qos)
        self.create_subscription(Bool, 'crossline/detected', self.crossline_callback, qos)

        # offset 토픽이 끊기면(카메라/추론 노드 다운) 정지시키는 워치독 타이머
        self.create_timer(0.1, self.watchdog_tick)

        self._lane_detected = False

        self.get_logger().info('lane_follower_node 시작 (cmd_vel 발행)')

    def detected_callback(self, msg: Bool):
        self._lane_detected = msg.data
        if not msg.data:
            self._lost_count += 1
        else:
            self._lost_count = 0

    def crossline_callback(self, msg: Bool):
        if self.stop_on_crossline and msg.data and self._crossline_stop_until is None:
            self.get_logger().info('Crossline 검출: 잠시 정지')
            self._crossline_stop_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.crossline_stop_duration)

    def offset_callback(self, msg: Float32):
        now = self.get_clock().now()
        self._last_msg_time = now

        # Crossline 정지 구간이면 그대로 정지 유지
        if self._crossline_stop_until is not None:
            if now < self._crossline_stop_until:
                self.publish_cmd(0.0, 0.0)
                return
            self._crossline_stop_until = None

        # 차선을 너무 오래 잃어버렸으면 정지 (그 자리에서 찾을 시간을 줌)
        if self._lost_count >= self.max_lost_frames:
            self.publish_cmd(0.0, 0.0)
            return

        offset = float(msg.data)
        dt = max((now - self._prev_time).nanoseconds / 1e9, 1e-3)
        d_offset = (offset - self._prev_offset) / dt

        angular_z = -(self.kp * offset + self.kd * d_offset)
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

        # 많이 꺾을수록(오프셋이 클수록) 속도를 줄여서 커브에서 안정적으로 돌게 함
        speed_scale = max(0.0, 1.0 - min(abs(offset), 1.0))
        linear_x = self.min_linear_speed + (self.linear_speed - self.min_linear_speed) * speed_scale

        self.publish_cmd(linear_x, angular_z)

        self._prev_offset = offset
        self._prev_time = now

    def watchdog_tick(self):
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if elapsed > self.watchdog_timeout:
            self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, linear_x: float, angular_z: float):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
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
        # 종료 시 로봇 정지
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()