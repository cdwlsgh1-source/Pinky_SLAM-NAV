# ROS
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # PoseStamped(목표 좌표 표현), PoseWithCovarianceStamped(불확실성 포함 좌표 — AMCL 초기 위치용)

# 파라미터 값 변경
#from rcl_interfaces.srv import SetParameters
#from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

import json
import math
import os
import time

POSE_FILE = os.path.expanduser('~/.pinky_last_pose.json') # 마지막 위치를 저장할 파일의 절대 경로를 미리 계산해서 상수로 고정


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def make_pose(navigator, x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def get_current_pose(navigator):
    """직전에 완료된 goToPose 액션의 마지막 feedback에서 실제(AMCL 보정) 위치를 읽어온다.
    아직 아무 액션도 실행된 적이 없으면 None을 반환한다."""
    feedback = navigator.getFeedback()
    if feedback is None:
        return None
    cp = feedback.current_pose.pose
    return (cp.position.x, cp.position.y, quat_to_yaw(cp.orientation))


def publish_initial_pose(navigator, x, y, yaw):
    """navigator.setInitialPose()는 covariance를 0으로 보내 AMCL이 이 값을 100% 정확하다고
    믿어버린다. 그러면 저장된 값이 조금만 틀려도 AMCL이 스스로 보정하지 못하고 오차가
    반복 실행마다 누적된다. 여기서는 covariance를 채운 PoseWithCovarianceStamped를 직접
    'initialpose'에 발행해서 AMCL이 센서 데이터로 스스로 보정할 여지를 남긴다.
    (covariance 값은 pinky_navigation/scripts/nav2_web_server.py의 set_initial_pose()와 동일)"""
    pub = navigator.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

    # 퍼블리셔 생성 직후 바로 publish하면 디스커버리가 안 끝나 메시지가 유실될 수 있어 잠깐 대기
    deadline = time.time() + 5.0
    while pub.get_subscription_count() == 0 and time.time() < deadline:
        time.sleep(0.1)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = navigator.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.06853891909122467
    pub.publish(msg)


def save_last_pose(x, y, yaw=0.0):
    with open(POSE_FILE, 'w') as f:
        json.dump({'x': x, 'y': y, 'yaw': yaw}, f)


def load_last_pose():
    if os.path.exists(POSE_FILE):
        with open(POSE_FILE) as f:
            data = json.load(f)
            data.setdefault('yaw', 0.0)  # 예전(yaw 없는) 파일 형식과 호환
            return data
    return None

# 파라미터 값 변경
#def set_inflation_radius(navigator, radius=0.3, costmap_names=('global_costmap', 'local_costmap')):
#    """
#    global_costmap / local_costmap 노드의 inflation_layer.inflation_radius 값을
#    런타임에 지정한 값으로 변경한다. (기본 파라미터 파일 값은 그대로 두고,
#    노드가 켜져 있는 동안에만 서비스 호출로 덮어씀)
#    """
#    for costmap_name in costmap_names:
#        service_name = f'/{costmap_name}/{costmap_name}/set_parameters'
#        client = navigator.create_client(SetParameters, service_name)
#
#        if not client.wait_for_service(timeout_sec=5.0):
#            print(f'[WARN] {service_name} 서비스가 존재하지 않습니다. (해당 costmap 미실행?)')
#            continue
#
#        param = Parameter()
#        param.name = 'inflation_layer.inflation_radius'
#        param.value = ParameterValue(
#            type=ParameterType.PARAMETER_DOUBLE,
#            double_value=float(radius)
#        )
#
#        request = SetParameters.Request()
#        request.parameters = [param]
#
#        future = client.call_async(request)
#        rclpy.spin_until_future_complete(navigator, future, timeout_sec=5.0)
#
#        if future.result() is not None:
#            results = future.result().results
#            if results and results[0].successful:
#                print(f'[OK] {costmap_name}.inflation_layer.inflation_radius -> {radius} 적용 완료')
#            else:
#                reason = results[0].reason if results else 'unknown'
#                print(f'[FAIL] {costmap_name} 파라미터 변경 실패: {reason}')
#        else:
#            print(f'[FAIL] {costmap_name} 파라미터 변경 요청 timeout')


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator() 
    home_pose = make_pose(navigator, 0.0, 0.0)

    # 추가
    skip_init = input('이미 RViz에서 초기 ㅟ치를 잡으셨나요? (y/n): ').strip().lower() == 'y'
    if not skip_init:
        last = load_last_pose()
        
        if last is None:
            publish_initial_pose(navigator, 0.0, 0.0, 0.0)
        else:
            publish_initial_pose(navigator, last['x'], last['y'], last['yaw'])
    # last 값에 아무것도 없으면,
    # 초기 위치를 (0, 0, 0)으로 잡고, last 값이 존재하면, last 값으로 초기 위치를 잡음.

    navigator.waitUntilNav2Active()              # bt_navigator가 Active 되거 전까지 대기
    #set_inflation_radius(navigator, radius=0.3)  # radius=0.15 -> 0.3 변경

    waypoints = [
        make_pose(navigator, 16.8795, -8.0572), 
        make_pose(navigator, 6.4920, -3.9554),
        make_pose(navigator, 6.37379, -0.64218)
    ]                                            # waypints 리스트 안에 좌표 객체 리스트 지정

    # <Waypoint 순차 이동 및 완료 대기 루프>
    for i, wp in enumerate(waypoints):
        navigator.goToPose(wp)                   # BasicNavigator.goToPose(wp) -> 0(i) 좌표(wp). / 1 좌표. / 2 좌표. 형식으로 표현됨 
        while not navigator.isTaskComplete():
            time.sleep(0.1)                      # 0.1초 Delay time
        result = navigator.getResult()           # BasicNavigator.GetResult()

        current = get_current_pose(navigator)    # 함수 변수지정
        if current is None:                      # feedback을 못 받은 경우에만 명령 좌표로 대체 (예전 동작과 동일한 안전망)
            print(f'[WARN] {i+1}번째 목표: 실제 위치를 못 읽어 명령 좌표로 저장합니다.')
            current = (wp.pose.position.x, wp.pose.position.y, 0.0)
        save_last_pose(*current)                 # *current는 튜플 언패킹

        if result == TaskResult.SUCCEEDED:
            print(f'{i+1}번째 목표 도착 성공')
        else:
            print(f'{i+1}번째 목표 실패: {result}')

    navigator.goToPose(home_pose)               # BasicNavigator,goToPose() - home_pose는 원점(0,0)으로 가라
    while not navigator.isTaskComplete():       # 로봇이 실제로 원점에 도착(또는 실패)할 때까지 0.1초마다 확인하며 대기
        time.sleep(0.1)
    result = navigator.getResult()

    current = get_current_pose(navigator)       # 함수를 변수로 지정
    if current is None:
        print('[WARN] 복귀 위치를 못 읽어 명령 좌표(0, 0)로 저장합니다.')
        current = (0.0, 0.0, 0.0)
    save_last_pose(*current)

    if result == TaskResult.SUCCEEDED:
        print('원래 자리로 복귀 완료')
    else:
        print(f'원래 자리로 복귀 실패: {result}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()