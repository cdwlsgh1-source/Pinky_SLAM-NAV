import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32


# ===============================================================
# 단일 인스턴스 마스크 -> ROI 내 x좌표 평균
# ===============================================================
def _instance_centroid_x(instance_mask: np.ndarray, roi_start: int, min_pixels: int):
    roi = instance_mask[roi_start:, :]
    # np.nonzero: roi에서 0이 아닌(=마스크가 칠해진) 픽셀들의 좌표를 (y좌표 배열, x좌표 배열)로 반환
    ys, xs = np.nonzero(roi)
    if len(xs) < min_pixels:
        return None
    return float(xs.mean())


# ===============================================================
# 차선 인스턴스 마스크 -> 중심 오프셋 계산
# ===============================================================
def extract_center_offset(
    instance_masks: np.ndarray,
    roi_ratio: float = 0.2,
    min_pixels: int = 15,
    assumed_half_lane_width_ratio: float = 0.35,
):
    """
    instance_masks: (N, H, W) 크기의, Lane 클래스로 검출된 '개별 인스턴스' 마스크들
                     (좌/우 차선이 각각 별도 인스턴스로 검출된다고 가정).
    roi_ratio: 화면 하단에서 몇 %를 관심영역(ROI)으로 볼지. 0.2 = 하단 20%.
    min_pixels: 인스턴스를 유효한 차선으로 인정할 최소 픽셀 수 (노이즈 제거용).
    assumed_half_lane_width_ratio: 한쪽 차선만 보일 때, 반대편 차선까지의 거리를
                     이미지 폭 대비 비율로 가정 (예: 0.35 = 이미지 폭의 35%).

    반환값: (offset, detected)
      offset   : -1.0(화면 완전 왼쪽) ~ 1.0(화면 완전 오른쪽), 0.0이 정중앙
      detected : 좌/우 중 최소 한쪽 차선을 찾았는지 여부

    핵심 아이디어: 모든 차선 픽셀을 하나로 합쳐 평균내지 않고, 각 인스턴스의
    중심 x를 구한 뒤 이미지 중앙 기준 좌/우로 나눠서, "좌측 차선 중심"과
    "우측 차선 중심"의 중간점을 실제 목표 중심으로 삼는다.
    """
    if instance_masks is None or len(instance_masks) == 0:
        return 0.0, False

    h, w = instance_masks.shape[1], instance_masks.shape[2]
    roi_start = int(h * (1 - roi_ratio))
    image_center_x = w / 2.0

    left_xs, right_xs = [], []
    for inst in instance_masks:
        cx = _instance_centroid_x(inst, roi_start, min_pixels)
        if cx is None:
            continue
        if cx < image_center_x:
            left_xs.append(cx)
        else:
            right_xs.append(cx)

    # 여러 개로 쪼개져 검출된 경우(점선 등) 같은 쪽끼리는 평균으로 대표값 하나로 합침
    # "X if 조건 else Y" 는 조건이 참이면 X, 거짓이면 Y (리스트가 비어있으면 None)
    left_x = float(np.mean(left_xs)) if left_xs else None
    right_x = float(np.mean(right_xs)) if right_xs else None

    half_lane_px = assumed_half_lane_width_ratio * w

    if left_x is not None and right_x is not None:
        # 양쪽 다 보임: 진짜 중간점
        lane_center_x = (left_x + right_x) / 2.0
    elif left_x is not None:
        # 왼쪽만 보임: 오른쪽 차선이 half_lane_px*2 만큼 떨어져 있다고 가정하고
        # 그 중간점(왼쪽 차선 + half_lane_px)을 목표 중심으로 삼는다.
        lane_center_x = left_x + half_lane_px
    elif right_x is not None:
        lane_center_x = right_x - half_lane_px
    else:
        return 0.0, False

    offset = (lane_center_x - image_center_x) / image_center_x
    # np.clip(값, 최소, 최대): 값이 범위를 벗어나면 최소/최대로 잘라냄 (여기선 -1.0~1.0로 제한)
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

    # ===============================================================
    # 초기화 (파라미터 로드, YOLO 모델 로드, 구독자/발행자 등록)
    # ===============================================================
    def __init__(self):
        super().__init__('lane_detector_node')

        # 파라미터 선언 (이름, 기본값)
        self.declare_parameter('model_path', '/home/jinho/dev_ws/pinky_slam_nav/pinky_camera/best.pt')
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('crossline_class_id', 0)
        self.declare_parameter('lane_class_id', 1)
        self.declare_parameter('lane_roi_ratio', 0.2)
        self.declare_parameter('min_lane_pixels', 15)
        self.declare_parameter('assumed_half_lane_width_ratio', 0.35)
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
        self.min_lane_pixels = self.get_parameter('min_lane_pixels').value
        self.assumed_half_lane_width_ratio = self.get_parameter('assumed_half_lane_width_ratio').value
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

    # ===============================================================
    # 메인 콜백 (이미지 수신 -> YOLO 추론 -> crossline/offset/디버그영상 발행)
    # ===============================================================
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

    # ===============================================================
    # Lane 인스턴스 좌/우 분리 -> 중간점 오프셋 계산
    # ===============================================================
    def compute_lane_offset(self, result):
        if result.masks is None or result.boxes is None:
            return 0.0, False

        cls_ids = result.boxes.cls.cpu().numpy().astype(int)
        masks_np = result.masks.data.cpu().numpy()  # (N, H, W), 모델에 따라 원본과 크기가 다를 수 있음

        # np.where(조건)[0]: cls_ids 중 조건(lane 클래스)을 만족하는 원소들의 인덱스만 배열로 반환
        lane_indices = np.where(cls_ids == self.lane_class_id)[0]
        if len(lane_indices) == 0:
            return 0.0, False

        # 인스턴스를 합치지 않고 그대로 넘긴다 (좌/우 차선을 분리해서 보기 위함)
        # masks_np[lane_indices]: lane 인스턴스들만 골라냄 -> 0.5 초과 여부로 0/1 이진 마스크로 변환
        lane_instance_masks = (masks_np[lane_indices] > 0.5).astype(np.uint8)

        # 마스크 해상도가 원본 프레임과 다르면 정규화된 offset 계산에는 영향 없음
        # (extract_center_offset은 mask 자체의 W를 기준으로 정규화하기 때문)
        return extract_center_offset(
            lane_instance_masks,
            self.lane_roi_ratio,
            self.min_lane_pixels,
            self.assumed_half_lane_width_ratio,
        )

    # ===============================================================
    # 디버그 영상 발행 (검출 결과를 그린 이미지를 JPEG로 인코딩해 publish)
    # ===============================================================
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


# ===============================================================
# 엔트리 포인트 (노드 생성 후 rclpy.spin으로 실행, 종료 시 정리)
# ===============================================================
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