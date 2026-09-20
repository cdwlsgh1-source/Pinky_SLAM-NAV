import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
import json, math, os, time, threading


class PinkyPatrolNode(Node):
    """
    [설계 핵심]
    - Nav2 액션(navigate_to_pose)은 전부 로컬(BasicNavigator)로 처리
      -> 로봇 내부에서 끝나는 통신이라 도메인 브릿지 불필요
    - 관제 PC와는 'patrol_cmd'(명령 수신) / 'patrol_status'(상태 송신)
      단 2개의 평범한 String 토픽으로만 통신
      -> 브릿지로 리매핑해야 할 대상이 이 2개뿐이라 관리가 단순해짐
    - waypoints는 로봇마다 다르므로 로봇별로 따로 관리
    """

    # 재시작해도 마지막 위치를 기억하기 위한 저장 파일 경로
    POSE_FILE = os.path.expanduser('~/.pinky_last_pose.json')

    POINTS = {
    "P1": (0.002,  0.001),   # 원점(0,0) 부근 — 시작점
    "P2": (0.598,  0.151),   # 원점과 우측 구간 사이 중간 지점
    "P3": (1.502, -0.400),   # 우측 아래 끝 (far right, lower)
    "P4": (1.000, -0.400),   # 중앙-우측 아래 (mid right, lower)
    "P5": (1.000,  0.100),   # 중앙-우측 위 (mid right, upper)
    "P6": (1.498,  0.100),   # 우측 위 끝 (far right, upper)
    "P7": (0.002, -0.597),   # 원점 아래쪽 (x는 원점과 거의 동일, y만 아래로)
	}
 
    # Pinky 1 경로: 원점 -> 중간지점 -> 우측 사각형 한 바퀴 -> 되돌아 원점
    # WAYPOINTS = [
    #     POINTS["P1"],  # 시작점 (원점 부근, 0.002, 0.001)
    #     POINTS["P2"],  # 중간 경유점 (0.598, 0.151)
    #     POINTS["P3"],  # 우측 아래 끝 (1.502, -0.400)
    #     POINTS["P4"],  # 중앙-우측 아래 (1.000, -0.400)
    #     POINTS["P5"],  # 중앙-우측 위 (1.000, 0.100)
    #     POINTS["P6"],  # 우측 위 끝 (1.498, 0.100)
    #     POINTS["P5"],  # 중앙-우측 위로 복귀 (1.000, 0.100)
    #     POINTS["P1"],  # 시작점으로 복귀 (0.002, 0.001)
    # ]

    # Pinky 2 경로: 원점 아래쪽 -> 우측 사각형 -> 원점 복귀
    WAYPOINTS = [
        POINTS["P7"],  # 시작점 (원점 아래쪽, 0.002, -0.597)
        POINTS["P3"],  # 우측 아래 끝 (1.502, -0.400)
        POINTS["P6"],  # 우측 위 끝 (1.498, 0.100)
        POINTS["P7"],  # 시작점 (원점 아래쪽, 0.002, -0.597)
    ]

    def __init__(self):
        super().__init__('pinky_patrol_node')

        # Nav2 액션 클라이언트 역할 + 퍼블리셔 생성 헬퍼 역할을 겸함
        self.navigator = BasicNavigator()

        # 관제 PC로 상태를 알리는 퍼블리셔
        self.status_pub = self.create_publisher(String, 'patrol_status', 10)
        # 관제 PC로부터 명령을 받는 구독자 (콜백은 짧게 유지!)
        self.create_subscription(String, 'patrol_cmd', self._cmd_cb, 10)

        # --- 스레드 간 공유 상태 플래그 ---
        self._running = False        # 현재 순찰 중인지
        self._stop_requested = False # stop 명령이 들어왔는지
        self._thread = None          # 순찰을 수행하는 백그라운드 스레드 핸들

        self.publish_status('IDLE', -1)

    def publish_status(self, state, wp_index, detail=''):
        """상태 문자열을 JSON으로 직렬화해서 patrol_status로 발행."""
        payload = json.dumps({'state': state, 'waypoint': wp_index,
                               'detail': detail, 'time': time.time()})
        self.status_pub.publish(String(data=payload))

    def _cmd_cb(self, msg):
        """
        [핵심 설계] 콜백은 절대 오래 걸리는 작업을 하지 않는다.
        rclpy.spin()이 이 콜백을 처리하는 동안 다른 이벤트(예: stop 명령)를
        못 받기 때문에, 실제 순찰은 스레드에 위임하고 콜백은 즉시 리턴한다.
        """
        cmd = msg.data.strip().lower()  # 공백/대소문자 차이 방지

        if cmd == 'start':
            if self._running:
                self.get_logger().warn('이미 순찰 중입니다. 명령 무시.')
                return
            self._stop_requested = False
            # daemon=True: 노드가 죽을 때 이 스레드도 함께 강제 종료되게 함
            self._thread = threading.Thread(target=self._run_patrol, daemon=True)
            self._thread.start()

        elif cmd == 'stop':
            if not self._running:
                self.get_logger().warn('순찰 중이 아닙니다. 명령 무시.')
                return
            self._stop_requested = True
            # 로컬 액션이므로 브릿지 없이 바로 취소 가능
            self.navigator.cancelTask()

        else:
            self.get_logger().warn(f'알 수 없는 명령: {cmd}')

    def quat_to_yaw(self, q):
        """
        쿼터니언(x,y,z,w) -> yaw(rad) 변환.
        asin/acos 대신 atan2를 쓰는 이유: 전체 -pi~pi 범위를 사분면 구분 없이
        올바르게 복원하기 위함 (asin/acos는 특정 구간에서 값이 꼬임).
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def make_pose(self, x, y, yaw=0.0):
        """yaw(rad) -> PoseStamped. 2D 로봇이라 z축 회전만 고려하면 충분."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # yaw -> quaternion 변환 공식 (z축 회전만 있는 경우의 축약형)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def get_current_pose(self):
        """Nav2가 이동 중 보내주는 feedback에서 현재 위치를 뽑아냄."""
        feedback = self.navigator.getFeedback()
        if feedback is None:
            return None
        cp = feedback.current_pose.pose
        return (cp.position.x, cp.position.y, self.quat_to_yaw(cp.orientation))

    def publish_initial_pose(self, x, y, yaw):
        """
        [구독자 대기 패턴 #1]
        퍼블리셔를 만들자마자 publish하면, 구독자(AMCL)가 아직 discovery
        되지 않았을 경우 메시지가 그냥 유실된다. initialpose는 1회성 메시지라
        유실되면 위치 추정이 완전히 틀어지므로, 구독자가 붙을 때까지 대기한다.
        """
        pub = self.navigator.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        deadline = time.time() + 5.0
        while pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.navigator.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # covariance는 6x6 행렬을 1차원 배열로 편 것. 대각선 성분만 채움:
        # [0]=x분산, [7]=y분산, [35]=yaw분산 (6*5+5=35)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.3
        pub.publish(msg)

    def spin_in_place(self, duration=4.0, angular_speed=0.5):
        """
        [구독자 대기 패턴 #2 - publish_initial_pose와 동일 구조]
        초기 위치 보정 후 제자리 회전을 시켜 AMCL이 방향을 더 잘 잡게 함.
        """
        cmd_pub = self.navigator.create_publisher(Twist, 'cmd_vel', 10)
        deadline = time.time() + 5.0
        while cmd_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)

        twist = Twist()
        twist.angular.z = angular_speed
        end_time = time.time() + duration
        while time.time() < end_time:
            cmd_pub.publish(twist)
            time.sleep(0.1)
        cmd_pub.publish(Twist())  # 정지 명령(모든 값 0)으로 마무리
        time.sleep(0.5)

    def save_last_pose(self, x, y, yaw=0.0):
        """다음 실행 시 이어서 시작할 수 있도록 마지막 위치를 파일에 저장."""
        with open(self.POSE_FILE, 'w') as f:
            json.dump({'x': x, 'y': y, 'yaw': yaw}, f)

    def load_last_pose(self):
        if os.path.exists(self.POSE_FILE):
            with open(self.POSE_FILE) as f:
                data = json.load(f)
                data.setdefault('yaw', 0.0)  # 구버전 파일 호환용 기본값
                return data
        return None

    def _run_patrol(self):
        """
        [별도 스레드에서 실행되는 메인 로직]
        rclpy.spin()을 막지 않아야 stop 명령을 도중에도 받을 수 있다.
        """
        self._running = True
        self.publish_status('STARTING', -1)

        # 1) 이전 위치가 있으면 초기 위치 보정부터 수행
        last = self.load_last_pose()
        if last is not None:
            start_x, start_y, start_yaw = last['x'], last['y'], last['yaw']
            self.publish_initial_pose(start_x, start_y, start_yaw)
            self.get_logger().info('초기 방향 보정을 위해 제자리 회전을 시작합니다...')
            self.spin_in_place(duration=4.0, angular_speed=0.5)
            self.get_logger().info('초기 방향 보정 완료.')

        # 2) Nav2 스택이 완전히 활성화될 때까지 대기
        self.navigator.waitUntilNav2Active()

        # 3) 웨이포인트 순회
        #    enumerate: 인덱스 i와 값 wp를 동시에 꺼내기 위함 (로그/상태 표시용)
        for i, wp in enumerate(self.WAYPOINTS):
            if self._stop_requested:
                self.publish_status('STOPPED', i)
                break

            pose = self.make_pose(*wp)  # *wp: 튜플 (x,y,yaw) -> 개별 인자로 언패킹

            # 최대 3회 재시도 (일시적 장애물/로컬라이제이션 튐 등에 대응)
            for attempt in range(3):
                self.navigator.goToPose(pose)

                # goToPose는 비동기이므로, 완료될 때까지 폴링하며 대기
                while not self.navigator.isTaskComplete():
                    if self._stop_requested:
                        self.navigator.cancelTask()
                        break
                    time.sleep(0.1)

                if self._stop_requested:
                    break

                result = self.navigator.getResult()

                # 도착 여부와 무관하게 현재 위치를 저장해 다음 재시작에 대비
                current = self.get_current_pose()
                if current is None:
                    current = (pose.pose.position.x, pose.pose.position.y, 0.0)
                self.save_last_pose(*current)

                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'{i+1}번째 목표 도착 성공 ({attempt+1}번째 시도)')
                    self.publish_status('MOVING', i + 1)
                    break  # 성공 -> 재시도 루프 탈출 (else 블록 스킵됨)
                else:
                    self.get_logger().warn(
                        f'{i+1}번째 목표 {attempt+1}번째 시도 실패: {result}, 재시도합니다')
                    self.publish_status('RETRY', i + 1, str(result))
            else:
                # for-else: break 없이 3번 다 돌았을 때 = 3회 전부 실패
                self.get_logger().error(f'{i+1}번째 목표 최종 실패 (3회 모두 실패)')
                self.publish_status('FAILED', i + 1)

            if self._stop_requested:
                self.publish_status('STOPPED', i)
                break
        else:
            # for-else: 바깥 루프도 break 없이 끝까지 돌았을 때 = 전체 완주
            self.publish_status('DONE', len(self.WAYPOINTS))

        self._running = False

def main(args=None):
    rclpy.init(args=args)
    node = PinkyPatrolNode()

    # rclpy.spin(node) 대신 전용 executor를 명시적으로 만들어 사용
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()   # patrol_cmd 명령을 계속 기다림
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
