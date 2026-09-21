#!/usr/bin/env python3
"""
camera_publisher_node.py

라즈베리파이 CSI 카메라(Picamera2)로 영상을 캡처해
ROS 2 토픽으로 발행하는 '카메라 송출' 모범 예시 노드.

핵심 설계 포인트
  1. 하드웨어 오픈은 노드 생성 시점에 재시도 로직과 함께 한 번만 수행
  2. 캡처 콜백은 절대 예외로 죽지 않도록 try/except로 감싼다
     (한 프레임 실패가 전체 노드 다운으로 이어지면 안 됨)
  3. 이미지 토픽은 BEST_EFFORT + depth=1 QoS로 발행
     (구독자가 느려도 큐에 쌓이지 않고 항상 최신 프레임만 유지 -> 지연 누적 방지)
  4. 파라미터에 설명(description)을 달아 `ros2 param describe`로 확인 가능하게 함
  5. destroy_node에서 카메라 자원을 반드시 해제
"""

import sys
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage


class CameraPublisherNode(Node):
    """CSI 카메라 영상을 <namespace>/camera/image_raw/compressed 로 발행."""

    def __init__(self):
        super().__init__('camera_publisher_node')

        # ---------------------------------------------------------
        # 1) 파라미터 선언 (설명을 붙여두면 `ros2 param describe`로 확인 가능)
        # ---------------------------------------------------------
        self.declare_parameter(
            'width', 640, ParameterDescriptor(description='캡처 가로 해상도(px)'))
        self.declare_parameter(
            'height', 480, ParameterDescriptor(description='캡처 세로 해상도(px)'))
        self.declare_parameter(
            'fps', 30, ParameterDescriptor(description='목표 프레임레이트'))
        self.declare_parameter(
            'jpeg_quality', 80,
            ParameterDescriptor(description='JPEG 압축 품질 (1~100, 높을수록 고품질/고용량)'))
        self.declare_parameter(
            'frame_id', 'camera_link',
            ParameterDescriptor(description='이미지 헤더에 들어갈 frame_id'))
        self.declare_parameter(
            'hflip', True, ParameterDescriptor(description='좌우 반전 여부'))
        self.declare_parameter(
            'vflip', True, ParameterDescriptor(description='상하 반전 여부'))
        self.declare_parameter(
            'topic_name', 'camera/image_raw/compressed',
            ParameterDescriptor(description='발행할 토픽 이름 (네임스페이스는 자동으로 붙음)'))

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.frame_id = self.get_parameter('frame_id').value
        self.hflip = self.get_parameter('hflip').value
        self.vflip = self.get_parameter('vflip').value
        self.topic_name = self.get_parameter('topic_name').value

        # ---------------------------------------------------------
        # 2) 카메라 오픈 (실패 시 재시도, 최종 실패하면 예외를 던져
        #    main()에서 노드를 안전하게 종료시킴)
        # ---------------------------------------------------------
        self.picam2 = None
        self._open_camera_with_retry(max_retries=3, retry_delay_sec=1.0)

        # ---------------------------------------------------------
        # 3) 발행자: 실시간 영상 스트림에 적합한 QoS
        #    - BEST_EFFORT: 패킷 손실보다 지연이 더 나쁨 (영상은 몇 프레임
        #      빠져도 티가 잘 안 나지만, 지연이 쌓이면 로봇 조작이 위험해짐)
        #    - depth=1, KEEP_LAST: 큐에 쌓지 않고 항상 최신 프레임만 유지
        # ---------------------------------------------------------
        image_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(CompressedImage, self.topic_name, image_qos)

        # 발행 상태 모니터링용 카운터
        self._frame_count = 0
        self._drop_count = 0

        # ---------------------------------------------------------
        # 4) 타이머로 주기적 캡처 (fps 기반)
        # ---------------------------------------------------------
        self.timer = self.create_timer(1.0 / self.fps, self._capture_and_publish)

        self.get_logger().info(
            f'카메라 퍼블리셔 시작: {self.width}x{self.height} @ {self.fps}fps '
            f'-> {self.pub.topic_name} (QoS: BEST_EFFORT, depth=1)')

    # ===============================================================
    # 카메라 오픈
    # ===============================================================
    def _open_camera_with_retry(self, max_retries: int, retry_delay_sec: float):
        last_exc = None
        for attempt in range(1, max_retries + 1):
            try:
                self._open_camera()
                return
            except Exception as e:  # 하드웨어 오픈 실패는 다양한 예외로 올 수 있음
                last_exc = e
                self.get_logger().warn(
                    f'카메라 오픈 실패 ({attempt}/{max_retries}): {e}')
                time.sleep(retry_delay_sec)
        # 재시도 다 실패 -> 상위(main)에서 처리하도록 예외 재발생
        raise RuntimeError(f'카메라를 열 수 없습니다 (재시도 {max_retries}회 실패): {last_exc}')

    def _open_camera(self):
        # picamera2는 로봇(라즈베리파이)에만 설치되어 있으므로 노드 생성 시점에 import.
        # PC에서 이 노드를 import만 해도 에러 나지 않도록 지연 import를 유지.
        from libcamera import Transform
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={'size': (self.width, self.height), 'format': 'RGB888'},
            transform=Transform(hflip=self.hflip, vflip=self.vflip),
            controls={'FrameDurationLimits': (int(1e6 / self.fps), int(1e6 / self.fps))},
        )
        self.picam2.configure(config)
        self.picam2.start()

    # ===============================================================
    # 캡처 + 발행 (타이머 콜백)
    # ===============================================================
    def _capture_and_publish(self):
        try:
            frame = self.picam2.capture_array('main')
        except Exception as e:
            # 카메라 드라이버 레벨 오류 - 한 프레임 건너뛰고 계속 진행.
            # (여기서 예외를 그대로 던지면 타이머 콜백 체인이 멈추고
            #  노드가 조용히 죽은 것처럼 보일 수 있음)
            self._drop_count += 1
            self.get_logger().warn(f'프레임 캡처 실패, 건너뜁니다: {e}')
            return

        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self._drop_count += 1
            self.get_logger().warn('JPEG 인코딩 실패, 프레임을 건너뜁니다')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub.publish(msg)

        self._frame_count += 1
        # 약 10초에 한 번 상태 로그 - 실전 운영 시 유용
        if self._frame_count % max(self.fps * 10, 1) == 0:
            self.get_logger().info(
                f'발행 중: {self._frame_count} 프레임 (누락 {self._drop_count})')

    # ===============================================================
    # 종료 처리
    # ===============================================================
    def destroy_node(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception as e:  # 종료 중 오류는 로그만 남기고 계속 진행
                self.get_logger().warn(f'카메라 종료 중 오류: {e}')
            finally:
                self.picam2 = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CameraPublisherNode()
    except Exception as e:
        print(f'[camera_publisher_node] 카메라를 열 수 없습니다: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()