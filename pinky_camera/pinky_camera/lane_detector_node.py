import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32


def extract_center_offset(mask: np.ndarray, roi_ratio: float = 0.2):
    """
    mask: (H, W) 크기의 바이너리(0/1) 세그멘테이션 마스크 (Lane 클래스만 합친 것).
    roi_ratio: 화면 하단에서 몇 %를 관심영역(ROI)으로 볼지. 0.2 = 하단 20%.

    반환값: (offset, detected)
      offset   : -1.0(화면 완전 왼쪽) ~ 1.0(화면 완전 오른쪽), 0.0이 정중앙
      detected : ROI 안에 차선 픽셀이 하나라도 있었는지 여부
    """
    h, w = mask.shape
    roi_start = int(h * (1 - roi_ratio))
    roi = mask[roi_start:h, :]

    ys, xs = np.nonzero(roi)
    if len(xs) == 0:
        return 0.0, False

    center_x = xs.mean()
    image_center_x = w / 2.0
    offset = (center_x - image_center_x) / image_center_x
    offset = float(np.clip(offset, -1.0, 1.0))
    return offset, True


class LaneDetectorNode(Node):
    """
    PC(또는 로봇)에서 실행. <namespace>/camera/image_raw/compressed 를 구독해 YOLO(best.pt, segment)로
    추론하고 결과를 발행한다.
      - <namespace>/lane/debug/compressed  : 마스크/박스를 그린 디버그 영상 (구독자가 있을 때만)
      - <namespace>/crossline/detected     : Crossline 검출 여부 (std_msgs/Bool)
      - <namespace>/lane/center_offset     : 차선 중심 오프셋 -1.0~1.0 (std_msgs/Float32)
      - <namespace>/lane/detected          : 이번 프레임에서 차선을 찾았는지 여부 (std_msgs/Bool)
    """

    def __init__(self):
        super().__init__('lane_detector_node')

        # 파라미터 선언 (이름, 기본값)
        self.declare_parameter('model_path', '/home/jinho/dev_ws/pinky_slam_nav/pinky_camera/best.pt')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('crossline_class_id', 0)
        self.declare_parameter('lane_class_id', 1)
        self.declare_parameter('lane_roi_ratio', 0.4)
        self.declare_parameter('debug_image', True)
        self.declare_parameter('jpeg_quality', 80)

        # 파라미터 값 읽기
        self.model_path = self.get_parameter('model_path').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf = self.get_parameter('conf').value
        self.device = self.get_parameter('device').value
        self.crossline_class_id = self.get_parameter('crossline_class_id').value
        self.lane_class_id = self.get_parameter('lane_class_id').value
        self.lane_roi_ratio = self.get_parameter('lane_roi_ratio').value
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
        self.offset_pub = self.create_publisher(Float32, 'lane/center_offset', 10)
        self.lane_detected_pub = self.create_publisher(Bool, 'lane/detected', 10)

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

        # ---- 임시 디버그: 1초마다 실시간 검출 상태 출력 ----
        self.get_logger().info(
            f'검출 클래스: {result.boxes.cls.tolist()}, '
            f'masks 있음: {result.masks is not None}, '
            f'conf: {result.boxes.conf.tolist() if result.boxes is not None else []}',
            throttle_duration_sec=1.0)
        # --------------------------------------------

        # 3. Crossline 검출 여부 발행
        detected_classes = result.boxes.cls.tolist()
        crossline_detected = self.crossline_class_id in detected_classes
        self.crossline_pub.publish(Bool(data=crossline_detected))

        # 4. Lane 중심 오프셋 계산 및 발행
        offset, lane_detected = self.compute_lane_offset(result)
        self.offset_pub.publish(Float32(data=offset))
        self.lane_detected_pub.publish(Bool(data=lane_detected))

        # 5. 디버그 영상 발행
        self.publish_debug_image(msg, result)

    def compute_lane_offset(self, result):
        """result.masks 중 Lane 클래스에 해당하는 것만 골라 하나로 합친 뒤 중심 오프셋을 계산한다."""
        if result.masks is None or result.boxes is None:
            return 0.0, False

        cls_ids = result.boxes.cls.cpu().numpy().astype(int)
        masks_np = result.masks.data.cpu().numpy()  # (N, H, W), 모델에 따라 원본과 크기가 다를 수 있음

        lane_indices = np.where(cls_ids == self.lane_class_id)[0]

        # ---- 임시 디버그 ----
        self.get_logger().info(
            f'lane_class_id={self.lane_class_id}, cls_ids={cls_ids.tolist()}, '
            f'lane_indices={lane_indices.tolist()}, masks_np.shape={masks_np.shape}, '
            f'lane_roi_ratio={self.lane_roi_ratio}',
            throttle_duration_sec=1.0)
        # ---- 여기까지 ----
        
        if len(lane_indices) == 0:
            return 0.0, False

        lane_mask = np.any(masks_np[lane_indices] > 0.5, axis=0).astype(np.uint8)

        # ---- 임시 디버그 ----
        h, w = lane_mask.shape
        roi_start = int(h * (1 - self.lane_roi_ratio))
        roi = lane_mask[roi_start:h, :]
        self.get_logger().info(
            f'lane_mask 전체 픽셀 수={lane_mask.sum()}, '
            f'ROI(y={roi_start}~{h}) 안 픽셀 수={roi.sum()}',
            throttle_duration_sec=1.0)
        # ---- 여기까지 ----


        # 마스크 해상도가 원본 프레임과 다르면 정규화된 offset 계산에는 영향 없음
        # (extract_center_offset은 mask 자체의 W를 기준으로 정규화하기 때문)
        return extract_center_offset(lane_mask, self.lane_roi_ratio)

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