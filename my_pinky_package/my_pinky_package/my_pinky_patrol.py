import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import json, math, os, time


class PinkyPatrol:
    """
    로봇이 마지막 위치를 기억했다가 재시작 시 그 위치로 초기화하고,
    정해진 웨이포인트를 순찰하는 클래스.
    """

    # 로봇이 마지막으로 있었던 위치(x, y, yaw)를 저장해두는 파일 경로
    # -> 프로그램을 껐다 켜도 "저번에 어디 있었는지"를 기억하기 위한 용도
    POSE_FILE = os.path.expanduser('~/.pinky_last_pose.json')

    def __init__(self):
        # ROS2 통신 초기화 + Nav2와 통신할 navigator 객체 생성
        rclpy.init()
        self.navigator = BasicNavigator()
        # self.navigator에 저장해두면 이후 모든 메서드가 인자로 안 받고도
        # self.navigator로 바로 접근 가능해짐

    # ------------------------------------------------------------------
    # 쿼터니언(orientation) -> yaw(라디안) 각도로 변환
    # ROS2는 회전을 쿼터니언(x,y,z,w)으로 표현하지만 사람은 각도(yaw)로 이해하는 게 편해서
    # 저장/출력용으로 다시 yaw 값으로 뽑아내는 변환 함수
    # ------------------------------------------------------------------
    def quat_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # 목표 지점(x, y, yaw)을 Nav2가 이해하는 PoseStamped 메시지로 만들어줌
    # goToPose()에 넘길 "가야 할 목표 좌표" 객체를 생성하는 헬퍼 메서드
    # ------------------------------------------------------------------
    def make_pose(self, x, y, yaw=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    # ------------------------------------------------------------------
    # 로봇의 "실제(AMCL로 보정된) 현재 위치"를 가져옴
    # 명령으로 보낸 목표 좌표가 아니라, 로봇이 실제로 도착한 좌표를 얻기 위해
    # 직전 goToPose 액션의 feedback 값을 읽어온다.
    # ------------------------------------------------------------------
    def get_current_pose(self):
        feedback = self.navigator.getFeedback()
        if feedback is None:
            return None
        cp = feedback.current_pose.pose
        return (cp.position.x, cp.position.y, self.quat_to_yaw(cp.orientation))

    # ------------------------------------------------------------------
    # AMCL(위치 추정기)에게 "로봇이 지금 대략 여기 있다"고 초기 위치를 알려줌
    # navigator.setInitialPose() 기본 함수 대신 직접 만든 이유:
    #   기본 함수는 covariance(불확실도)를 0으로 보내서 AMCL이 그 값을 100% 정답이라 믿어버림
    #   -> 저장된 좌표가 조금만 틀려도 스스로 보정을 못 하고 오차가 누적됨
    #   여기서는 covariance를 일부러 채워서 AMCL이 센서로 스스로 보정할 여지를 남김
    #
    # [수정] yaw(방향) covariance를 기존보다 넉넉하게 키움
    #   -> 실제 로봇은 껐다 켜지는 사이에(도킹 각도가 미세하게 다르거나, 손으로 옮기거나 등)
    #      저장된 yaw와 실제 방향이 몇 도씩 어긋나는 경우가 흔함
    #      기존 값(0.0685)은 RViz "2D Pose Estimate" 기본값 수준으로 너무 타이트해서,
    #      저장된 yaw가 조금만 틀려도 AMCL이 그 오차를 스스로 못 고치고 그대로 굳어버림
    #      (=RViz에서 맵/벽이 라이다 스캔과 회전되어 어긋나 보이는 현상의 주 원인)
    # ------------------------------------------------------------------
    def publish_initial_pose(self, x, y, yaw):
        pub = self.navigator.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        # 퍼블리셔 생성 직후 바로 publish하면 디스커버리(구독자 연결)가 안 끝나
        # 메시지가 유실될 수 있어 구독자가 붙을 때까지 잠깐 대기
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

        # 위치/각도에 대한 불확실도(covariance) 값. 0이 아니게 채워서
        # AMCL이 센서 데이터를 참고해 스스로 보정하도록 유도
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.3  # [수정] yaw 분산을 키워 방향 오차를 스스로 더 잘 보정하게 함
        pub.publish(msg)

    # ------------------------------------------------------------------
    # 초기 위치 publish 직후, 로봇을 제자리에서 회전시켜
    # AMCL이 여러 각도의 라이다 스캔을 받아 실제 방향으로 스스로 수렴하도록 도와주는 메서드
    #
    # 왜 필요한가:
    #   publish_initial_pose()는 "이 방향일 것이다"라는 추측값만 알려줄 뿐,
    #   그 추측이 실제와 다르면(=저장된 yaw와 실제 로봇 방향이 어긋나면) 가만히 있는 것만으로는
    #   AMCL의 particle filter가 스스로 정답 방향을 찾기 어려움 (관측 각도가 한 방향뿐이라서)
    #   제자리에서 한 바퀴(또는 좌우로) 돌려주면 다양한 각도의 스캔 데이터가 들어오면서
    #   particle filter가 실제 방향으로 빠르게 수렴함
    # ------------------------------------------------------------------
    def spin_in_place(self, duration=4.0, angular_speed=0.5):
        cmd_pub = self.navigator.create_publisher(Twist, 'cmd_vel', 10)

        # cmd_vel 구독자(로봇 드라이버)가 붙을 때까지 잠깐 대기 (initial pose와 동일한 이유)
        deadline = time.time() + 5.0
        while cmd_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)

        twist = Twist()
        twist.angular.z = angular_speed

        end_time = time.time() + duration
        while time.time() < end_time:
            cmd_pub.publish(twist)
            time.sleep(0.1)

        # 회전이 끝나면 반드시 정지 명령을 보내 로봇이 계속 돌지 않도록 함
        cmd_pub.publish(Twist())
        time.sleep(0.5)  # 정지 명령이 실제로 반영될 시간을 잠깐 줌

    # ------------------------------------------------------------------
    # 마지막 위치(x, y, yaw)를 JSON 파일로 저장
    # ------------------------------------------------------------------
    def save_last_pose(self, x, y, yaw=0.0):
        with open(self.POSE_FILE, 'w') as f:
            json.dump({'x': x, 'y': y, 'yaw': yaw}, f)

    # ------------------------------------------------------------------
    # 저장해둔 마지막 위치를 불러옴 (파일 없으면 None)
    # ------------------------------------------------------------------
    def load_last_pose(self):
        if os.path.exists(self.POSE_FILE):
            with open(self.POSE_FILE) as f:
                data = json.load(f)
                data.setdefault('yaw', 0.0)  # 예전(yaw 없는) 파일 형식과 호환
                return data
        return None

    # ------------------------------------------------------------------
    # 순찰 실행 메서드 (기존 main() 함수의 로직이 여기로 옮겨옴)
    # ------------------------------------------------------------------
    def run(self):
        # 저번 종료 시점 위치 불러오기
        last = self.load_last_pose()

        if last is None:
            start_x, start_y, start_yaw = 0.0, 0.0, 0.0
            home_pose = self.make_pose(start_x, start_y, start_yaw)
            # 저장된 위치가 없으면 RViz에서 이미 설정한 initialpose를 그대로 사용 (덮어쓰지 않음)
        else:
            start_x, start_y, start_yaw = last['x'], last['y'], last['yaw']
            home_pose = self.make_pose(start_x, start_y, start_yaw)
            self.publish_initial_pose(start_x, start_y, start_yaw)  # 이전 기록 있을 때만 발행

            # [신규 추가] 저장된 yaw와 실제 로봇 방향이 어긋나 있을 수 있으므로,
            # 제자리 회전으로 AMCL이 실제 방향으로 스스로 수렴하도록 함
            # (RViz에서 맵/벽이 라이다 스캔과 회전되어 어긋나 보이는 현상 방지)
            print('[INFO] 초기 방향 보정을 위해 제자리 회전을 시작합니다...')
            self.spin_in_place(duration=4.0, angular_speed=0.5)
            print('[INFO] 초기 방향 보정 완료.')

        # Nav2 스택(경로계획/제어 등)이 완전히 켜질 때까지 대기
        self.navigator.waitUntilNav2Active()

        # 순서대로 방문할 웨이포인트(목표 지점) 목록 정의
        # yaw는 반시계 방향(CCW)이 양수 → 양수(+90°)는 왼쪽, 음수(-90°)는 오른쪽
        waypoints = [
            self.make_pose(0.08, 0.02, 0),                # 1번째 목표(Charging), (정면 0도, 0[rad])
            self.make_pose(0.59, 0.09, -1.57),           # 2번째 목표, (오른쪽 90도, -1.57[rad])
            self.make_pose(1.49, -0.45, 3.14),            # 3번째 목표, (뒤로 180도, 3.14[rad])

            self.make_pose(1.00, -0.45, 1.57),            # 4번째 목표, (왼쪽 90도, 1.57[rad])

            self.make_pose(1.49, -0.03, 3.14),             # 5번째 목표, (뒤로 180도, 3.14[rad])
            self.make_pose(0.07, -0.68, 1.57),            # 6번째 목표, (왼쪽 90도, 1.57[rad])
            self.make_pose(0.08, 0.02, 0),                # 1번째 목표(Charging) (복귀, 정면 0도, 0[rad])
        ]

        # 웨이포인트를 하나씩 순회하며 이동 -> 완료 대기 -> 결과 확인 -> 위치 저장
        for i, wp in enumerate(waypoints):
            for attempt in range(3):
                self.navigator.goToPose(wp)                    # 목표 전송 (비동기 액션 시작) - 이동 명령
                while not self.navigator.isTaskComplete():     # 도착/실패할 때까지 폴링 대기
                    time.sleep(0.1)
                result = self.navigator.getResult()            # SUCCEEDED / FAILED / CANCELED 등

                # 실제 도착 위치를 기록 (feedback 못 받으면 명령 좌표로 대체하는 안전망)
                current = self.get_current_pose()
                if current is None:
                    print(f'[WARN] {i+1}번째 목표: 실제 위치를 못 읽어 명령 좌표로 저장합니다.')
                    current = (wp.pose.position.x, wp.pose.position.y, 0.0)
                self.save_last_pose(*current)

                if result == TaskResult.SUCCEEDED:
                    print(f'{i+1}번째 목표 도착 성공 ({attempt+1}번째 시도)')
                    break
                else:
                    print(f'{i+1}번째 목표 {attempt+1}번째 시도 실패: {result}, 재시도합니다')
            else:
                print(f'{i+1}번째 목표 최종 실패 (3회 모두 실패)')

        # ROS2 통신 종료
        rclpy.shutdown()


def main(args=None):
    patrol = PinkyPatrol()   # 객체 생성 (이때 __init__ 실행됨)
    patrol.run()             # 실제 순찰 로직 실행


if __name__ == '__main__':
    main()