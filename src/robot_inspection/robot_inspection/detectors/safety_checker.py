"""
Safety Compliance Checker — helmet, vest, zone intrusion, fire/smoke.

Detects safety violations in industrial environments using YOLOv8 +
geometric zone analysis.
"""
import logging
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inspection_msgs.msg import SafetyCheck, Alert

logger = logging.getLogger(__name__)

SAFETY_CLASSES = ['person', 'helmet', 'vest', 'fire', 'smoke']
# Mapping: what class each detection represents
VIOLATION_CHECKS = {
    'helmet': 'person_without_helmet',
    'vest': 'person_without_vest',
    'zone_intrusion': 'unauthorized_entry',
    'fire': 'fire_detected',
    'smoke': 'smoke_detected',
}


class SafetyCheckerNode(Node):
    """ROS2 node for safety compliance checking in industrial environments."""

    def __init__(self):
        super().__init__('safety_checker')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('processing_rate', 3.0)  # Hz
        self.declare_parameter('enable_mock', True)
        self.declare_parameter('restricted_zones', '[]')  # JSON polygon array

        model_path = self.get_parameter('model_path').value
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        self._iou_thresh = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        rate = self.get_parameter('processing_rate').value
        enable_mock = self.get_parameter('enable_mock').value

        # Parse restricted zones
        import json
        zones_raw = self.get_parameter('restricted_zones').value
        try:
            self._restricted_zones = json.loads(zones_raw)
        except (json.JSONDecodeError, TypeError):
            self._restricted_zones = []
        self._zone_labels = [z.get('label', f'zone_{i}')
                             for i, z in enumerate(self._restricted_zones)]

        # Inference engine
        from ..models.inference_engine import InferenceEngine
        from ..utils.preprocessor import ImagePreprocessor
        if not model_path and enable_mock:
            model_path = '__mock__'
        self._engine = InferenceEngine(model_path, device=device)
        self._preprocessor = ImagePreprocessor(target_size=self._engine.input_size)
        self._bridge = CvBridge()

        # State
        self._last_process_time = 0.0
        self._process_interval = 1.0 / rate if rate > 0 else 0.0

        # Publishers
        self._result_pub = self.create_publisher(
            SafetyCheck, '/inspection/safety/result', 10)
        self._alert_pub = self.create_publisher(
            Alert, '/inspection/safety/alert', 10)
        self._annotated_pub = self.create_publisher(
            Image, '/inspection/safety/annotated', 10)

        # Subscriber
        self._image_sub = self.create_subscription(
            Image, '/camera/rgb', self._image_callback, 10,
            callback_group=ReentrantCallbackGroup())

        if not self._engine.is_mock:
            self._engine.warmup()

        self.get_logger().info(
            f'SafetyChecker ready (device={self._engine.device}, '
            f'rate={rate}Hz, zones={len(self._restricted_zones)}, '
            f'mock={self._engine.is_mock})')

    def _image_callback(self, msg: Image):
        """Process camera images for safety compliance."""
        now = time.time()
        if now - self._last_process_time < self._process_interval:
            return
        self._last_process_time = now

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self._engine.is_mock:
                detections = self._mock_detect(image)
            else:
                tensor, scale, (pad_x, pad_y) = self._preprocessor.preprocess(image)
                result = self._engine.infer(tensor)
                detections = self._parse_detections(
                    result.outputs[0], scale, pad_x, image.shape)

            # Run zone intrusion check
            zone_violations = self._check_zone_intrusion(detections, image.shape)

            # Run safety gear checks
            gear_violations = self._check_safety_gear(detections)

            # Combine results
            all_violations = zone_violations + gear_violations

            if all_violations:
                self._publish_violations(all_violations, detections, msg.header, image)

            # Annotated output
            from ..utils.visualization import draw_detections, draw_zones
            annotated = draw_detections(image, detections + all_violations, 'safety')
            if self._restricted_zones:
                zone_polys = [np.array(z['points'], dtype=np.int32)
                              for z in self._restricted_zones]
                annotated = draw_zones(annotated, zone_polys, self._zone_labels)

            ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self._annotated_pub.publish(ann_msg)

        except Exception as e:
            self.get_logger().error(f'Safety check error: {e}', throttle_duration_sec=5.0)

    def _parse_detections(self, output: np.ndarray,
                          scale: float, padding: tuple,
                          image_shape: tuple) -> list[dict]:
        """Parse YOLO outputs into detection dicts."""
        detections = []
        if output.ndim == 3:
            dets = output[0]
        elif output.ndim == 2:
            dets = output
        else:
            return detections

        pad_x, pad_y = padding
        h, w = image_shape[:2]

        for det in dets:
            if len(det) < 6:
                continue
            x1, y1, x2, y2 = det[:4]
            conf = float(det[4])
            class_id = int(det[5])

            if conf < self._conf_thresh:
                continue
            if class_id >= len(SAFETY_CLASSES):
                continue

            x1 = max(0, (x1 - pad_x) / scale)
            y1 = max(0, (y1 - pad_y) / scale)
            x2 = min(w, (x2 - pad_x) / scale)
            y2 = min(h, (y2 - pad_y) / scale)

            detections.append({
                'bbox': [float(x1), float(y1), float(x2), float(y2)],
                'label': SAFETY_CLASSES[class_id],
                'confidence': conf,
                'center': [(x1 + x2) / 2, (y1 + y2) / 2],
            })

        return detections

    def _check_safety_gear(self, detections: list[dict]) -> list[dict]:
        """Check if detected persons are wearing required safety gear.

        Strategy: For each person bbox, check if there's an overlapping
        helmet/vest detection within the upper portion of the person box.
        """
        violations = []
        persons = [d for d in detections if d['label'] == 'person']
        helmets = [d for d in detections if d['label'] == 'helmet']
        vests = [d for d in detections if d['label'] == 'vest']

        for person in persons:
            px1, py1, px2, py2 = person['bbox']
            person_h = py2 - py1

            # Check helmet (upper 30% of person bbox)
            has_helmet = False
            for helm in helmets:
                hx1, hy1, hx2, hy2 = helm['bbox']
                if (hx1 < px2 and hx2 > px1 and
                        hy1 < py1 + person_h * 0.35 and hy2 > py1):
                    iou = self._compute_iou(person['bbox'], helm['bbox'])
                    if iou > 0.1:
                        has_helmet = True
                        break

            if not has_helmet:
                violations.append({
                    'label': 'helmet_missing',
                    'confidence': person['confidence'],
                    'bbox': person['bbox'],
                    'severity': 'major',
                })

            # Check vest (middle 40% of person bbox)
            has_vest = any(
                self._compute_iou(person['bbox'], v['bbox']) > 0.15
                for v in vests)
            if not has_vest:
                violations.append({
                    'label': 'vest_missing',
                    'confidence': person['confidence'],
                    'bbox': person['bbox'],
                    'severity': 'minor',
                })

        return violations

    def _check_zone_intrusion(self, detections: list[dict],
                              image_shape: tuple) -> list[dict]:
        """Check if any detected persons are inside restricted zones."""
        if not self._restricted_zones:
            return []

        violations = []
        persons = [d for d in detections if d['label'] == 'person']

        for person in persons:
            cx, cy = person['center']
            for zone_idx, zone in enumerate(self._restricted_zones):
                points = np.array(zone['points'], dtype=np.float32)
                if self._point_in_polygon(cx, cy, points):
                    violations.append({
                        'label': 'zone_intrusion',
                        'confidence': person['confidence'],
                        'bbox': person['bbox'],
                        'severity': 'critical' if zone.get('restricted', True) else 'minor',
                        'zone': self._zone_labels[zone_idx] if zone_idx < len(self._zone_labels) else f'zone_{zone_idx}',
                    })

        return violations

    @staticmethod
    def _point_in_polygon(x: float, y: float, polygon: np.ndarray) -> bool:
        """Check if a point is inside a polygon using ray casting."""
        return cv2.pointPolygonTest(
            polygon.reshape(-1, 1, 2).astype(np.float32), (x, y), False) >= 0

    @staticmethod
    def _compute_iou(bbox_a: list[float], bbox_b: list[float]) -> float:
        """Compute Intersection-over-Union between two bboxes."""
        x1 = max(bbox_a[0], bbox_b[0])
        y1 = max(bbox_a[1], bbox_b[1])
        x2 = min(bbox_a[2], bbox_b[2])
        y2 = min(bbox_a[3], bbox_b[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area_a = (bbox_a[2] - bbox_a[0]) * (bbox_a[3] - bbox_a[1])
        area_b = (bbox_b[2] - bbox_b[0]) * (bbox_b[3] - bbox_b[1])
        union = area_a + area_b - inter + 1e-6

        return inter / union

    def _mock_detect(self, image: np.ndarray) -> list[dict]:
        """Generate mock safety detections for simulation testing."""
        import random
        random.seed(int(time.time() * 1000) % 10000)
        h, w = image.shape[:2]

        detections = []

        # 60% chance of seeing 1-3 people
        if random.random() < 0.6:
            num_people = random.randint(1, 3)
            for _ in range(num_people):
                cx = random.randint(w // 4, 3 * w // 4)
                cy = random.randint(h // 3, 2 * h // 3)
                bw, bh = random.randint(60, 120), random.randint(120, 220)
                detections.append({
                    'bbox': [float(cx - bw // 2), float(cy - bh // 2),
                             float(cx + bw // 2), float(cy + bh // 2)],
                    'label': 'person',
                    'confidence': random.uniform(0.7, 0.95),
                    'center': [float(cx), float(cy)],
                })

        # 70% of people have helmets, 50% have vests
        for det in detections:
            if det['label'] == 'person':
                if random.random() < 0.7:
                    x1, y1, x2, y2 = det['bbox']
                    detections.append({
                        'bbox': [x1 - 5, y1 - 5, x2 + 5, y1 + (y2 - y1) * 0.3],
                        'label': 'helmet',
                        'confidence': random.uniform(0.6, 0.9),
                        'center': [(x1 + x2) / 2, y1 + (y2 - y1) * 0.15],
                    })
                if random.random() < 0.5:
                    detections.append({
                        'bbox': det['bbox'],
                        'label': 'vest',
                        'confidence': random.uniform(0.6, 0.9),
                        'center': det['center'],
                    })

        return detections

    def _publish_violations(self, violations: list[dict],
                            all_detections: list[dict],
                            header, image: np.ndarray):
        """Publish safety violations and alerts."""
        for v in violations:
            msg = SafetyCheck()
            msg.header = header
            msg.check_type = v['label']
            msg.status = 'violation'
            msg.description = f"Safety violation: {v['label']}"
            msg.confidence = float(v['confidence'])

            if 'bbox' in v:
                x1, y1, x2, y2 = [int(c) for c in v['bbox']]
                msg.evidence_image = self._bridge.cv2_to_imgmsg(
                    image[y1:y2, x1:x2], encoding='bgr8')

            # Count persons
            msg.person_count = sum(
                1 for d in all_detections if d['label'] == 'person')
            msg.violation_count = 1

            self._result_pub.publish(msg)

            # Alert for critical violations
            severity = v.get('severity', 'major')
            if severity in ('critical', 'major'):
                alert = Alert()
                alert.header = header
                alert.severity = 2 if severity == 'critical' else 1
                alert.alert_type = 'safety'
                alert.message = f"{severity.upper()} safety violation: {v['label']}"
                alert.suggested_actions = [
                    'Escort personnel out of restricted zone',
                    'Verify safety gear compliance',
                    'Log incident in safety report',
                ]
                self._alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
