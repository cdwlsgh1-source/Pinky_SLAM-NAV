import sys

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraNode(Node):
    """
    로봇(라즈베리파이)에서 실행. CSI 카메라(picamera2)로 프레임을 캡처해
    <namespace>/camera/image_raw/compressed (sensor_msgs/CompressedImage, JPEG)로 발행한다.
    라인 인식(best.pt) 노드는 이 토픽을 구독한다.
    """

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('frame_id', 'camera_link')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.frame_id = self.get_parameter('frame_id').value

        self.picam2 = None
        self._open_camera()

        self.pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.timer = self.create_timer(1.0 / self.fps, self._capture_cb)
        self.get_logger().info(
            f'카메라 시작: {self.width}x{self.height} @ {self.fps}fps '
            f'-> {self.pub.topic_name}')

    def _open_camera(self):
        # picamera2는 로봇에만 설치되어 있으므로 노드 생성 시점에 import
        from picamera2 import Picamera2

        self.picam2 = Picamera2()
        config = self.picam2.create_video_configuration(
            main={'size': (self.width, self.height), 'format': 'RGB888'},
            controls={'FrameDurationLimits': (int(1e6 / self.fps), int(1e6 / self.fps))},
        )
        self.picam2.configure(config)
        self.picam2.start()

    def _capture_cb(self):
        # picamera2의 'RGB888'은 메모리상 BGR 순서라 cv2에 그대로 사용 가능
        frame = self.picam2.capture_array('main')

        ok, buf = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn('JPEG 인코딩 실패, 프레임을 건너뜁니다')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        if self.picam2 is not None:
            try:
                self.picam2.stop()
                self.picam2.close()
            except Exception as e:  # 종료 중 오류는 로그만 남기고 계속 진행
                self.get_logger().warn(f'카메라 종료 중 오류: {e}')
            self.picam2 = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
    except Exception as e:
        # 다른 프로세스가 카메라를 점유했거나, 카메라 미인식/picamera2 미설치 등
        print(f'[camera_node] 카메라를 열 수 없습니다: {e}', file=sys.stderr)
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
