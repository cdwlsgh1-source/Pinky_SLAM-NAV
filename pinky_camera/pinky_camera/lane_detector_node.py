import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool


class LaneDetectorNode(Node):
    """
    PC에서 실행. <namespace>/camera/image_raw/compressed 를 구독해 YOLO(best.pt, segment)로
    추론하고 결과를 발행한다.
      - <namespace>/lane/debug/compressed : 마스크/박스를 그린 디버그 영상 (구독자가 있을 때만)
      - <namespace>/crossline/detected    : Crossline 검출 여부 (std_msgs/Bool)
    """

    def __init__(self):
        super().__init__('lane_detector_node')

        # 파라미터 선언 (이름, 기본값)
        self.declare_parameter('model_path', '/home/jinho/dev_ws/best.pt')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('crossline_class_id', 0)
        self.declare_parameter('debug_image', True)
        self.declare_parameter('jpeg_quality', 80)

        # 파라미터 값 읽기
        self.model_path = self.get_parameter('model_path').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf = self.get_parameter('conf').value
        self.device = self.get_parameter('device').value
        self.crossline_class_id = self.get_parameter('crossline_class_id').value
        self.debug_image = self.get_parameter('debug_image').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # ultralytics(torch)는 import가 무거우므로 노드 생성 시점에 import
        from ultralytics import YOLO
        self.model = YOLO(self.model_path)

        # 추론이 느려도 지연이 쌓이지 않도록 최신 프레임 1개만 유지
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 구독자 / 발행자
        self.sub = self.create_subscription(
            CompressedImage, 'camera/image_raw/compressed', self.image_callback, qos)
        self.debug_pub = self.create_publisher(
            CompressedImage, 'lane/debug/compressed', 10)
        self.crossline_pub = self.create_publisher(Bool, 'crossline/detected', 10)

        self.get_logger().info(
            f'모델 로드 완료: {self.model_path} (task={self.model.task}, '
            f'classes={self.model.names}) {self.sub.topic_name} 구독 중')

    def image_callback(self, msg):
        # 1. JPEG -> OpenCV 이미지
        jpeg_bytes = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('JPEG 디코딩 실패, 프레임을 건너뜁니다')
            return

        # 2. YOLO 추론
        results = self.model.predict(
            frame, imgsz=self.imgsz, conf=self.conf, device=self.device, verbose=False)
        result = results[0]

        # 3. Crossline 검출 여부 발행
        detected_classes = result.boxes.cls.tolist()
        detected = self.crossline_class_id in detected_classes
        self.crossline_pub.publish(Bool(data=detected))

        # 4. 디버그 영상 발행
        self.publish_debug_image(msg, result)

    def publish_debug_image(self, msg, result):
        # 디버그 옵션이 꺼져 있거나 구독자가 없으면 plot/JPEG 인코딩을 생략해 CPU 절약
        if not self.debug_image:
            return
        if self.debug_pub.get_subscription_count() == 0:
            return

        plotted = result.plot()
        ok, buf = cv2.imencode(
            '.jpg', plotted, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self.get_logger().warn('디버그 JPEG 인코딩 실패')
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)

    # 노드 생성 (모델 파일 없음/손상, ultralytics 미설치 등이면 실패)
    try:
        node = LaneDetectorNode()
    except Exception as e:
        print(f'[lane_detector_node] 노드를 시작할 수 없습니다: {e}', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    # 실행
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
