"""
Product Defect Detector — surface defects, dimensions, assembly quality.

Detects: scratches, cracks, deformation, stains, missing parts.
Uses YOLOv8 ONNX model for object detection + traditional CV for fine-grained analysis.
"""
import logging
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inspection_msgs.msg import DefectDetection, Alert

from ..models.inference_engine import InferenceEngine
from ..utils.preprocessor import ImagePreprocessor
from ..utils.visualization import draw_detections

logger = logging.getLogger(__name__)

# COCO-style class names for industrial defects
DEFECT_CLASSES = ['scratch', 'crack', 'deformation', 'stain', 'missing_part']
SEVERITY_MAP = {0: 'minor', 1: 'minor', 2: 'major', 3: 'critical', 4: 'major'}


class DefectDetectorNode(Node):
    """ROS2 node for product defect detection on production lines."""

    def __init__(self):
        super().__init__('defect_detector')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('processing_rate', 5.0)  # Hz
        self.declare_parameter('enable_mock', True)

        model_path = self.get_parameter('model_path').value
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        self._iou_thresh = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        rate = self.get_parameter('processing_rate').value
        enable_mock = self.get_parameter('enable_mock').value

        # Inference engine
        if not model_path and enable_mock:
            model_path = '__mock__'  # Force mock mode
        self._engine = InferenceEngine(model_path, device=device)
        self._preprocessor = ImagePreprocessor(target_size=self._engine.input_size)
        self._bridge = CvBridge()

        # State
        self._last_process_time = 0.0
        self._process_interval = 1.0 / rate if rate > 0 else 0.0
        self._frame_count = 0

        # Publishers
        self._result_pub = self.create_publisher(
            DefectDetection, '/inspection/defect/result', 10)
        self._alert_pub = self.create_publisher(
            Alert, '/inspection/defect/alert', 10)
        self._annotated_pub = self.create_publisher(
            Image, '/inspection/defect/annotated', 10)

        # Subscriber
        self._image_sub = self.create_subscription(
            Image, '/camera/rgb', self._image_callback, 10,
            callback_group=ReentrantCallbackGroup())

        # Warmup
        if not self._engine.is_mock:
            self._engine.warmup()

        self.get_logger().info(
            f'DefectDetector ready (device={self._engine.device}, '
            f'rate={rate}Hz, mock={self._engine.is_mock})')

    def _image_callback(self, msg: Image):
        """Process incoming camera images at configured rate."""
        now = time.time()
        if now - self._last_process_time < self._process_interval:
            return
        self._last_process_time = now

        try:
            # Convert ROS image to numpy
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Enhance for better defect visibility
            from ..utils.preprocessor import ImagePreprocessor
            enhanced = ImagePreprocessor.enhance_for_inspection(image)

            # Preprocess and infer
            tensor, scale, (pad_x, pad_y) = self._preprocessor.preprocess(enhanced)
            result = self._engine.infer(tensor)

            # Parse detections
            detections = self._parse_detections(
                result.outputs[0], scale, pad_x, image.shape)

            # Publish results
            if detections:
                self._publish_results(detections, msg.header, image)

            # Annotated output
            annotated = draw_detections(image, detections, 'defect')
            ann_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ann_msg.header = msg.header
            self._annotated_pub.publish(ann_msg)

            self._frame_count += 1
            if self._frame_count % 100 == 0:
                self.get_logger().debug(
                    f'Processed {self._frame_count} frames, '
                    f'{len(detections)} defects detected')

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}', throttle_duration_sec=5.0)

    def _parse_detections(self, output: np.ndarray,
                          scale: float, padding: tuple,
                          image_shape: tuple) -> list[dict]:
        """Parse raw ONNX output into structured detection results.

        Handles YOLOv8 output format: [batch, num_boxes, 4+1+num_classes].
        Output format varies by model export; this handles common variations.
        """
        detections = []

        # Handle different output formats
        if output.ndim == 3:
            # [batch, num_dets, 6] with format [x1, y1, x2, y2, conf, class]
            dets = output[0]  # First batch
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
            if class_id >= len(DEFECT_CLASSES):
                continue

            # Undo letterbox padding
            x1 = (x1 - pad_x) / scale
            y1 = (y1 - pad_y) / scale
            x2 = (x2 - pad_x) / scale
            y2 = (y2 - pad_y) / scale

            # Clamp to image bounds
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w, x2), min(h, y2)

            label = DEFECT_CLASSES[class_id]
            severity = SEVERITY_MAP.get(class_id, 'minor')

            detections.append({
                'bbox': [float(x1), float(y1), float(x2), float(y2)],
                'label': label,
                'confidence': conf,
                'severity': severity,
                'area': float((x2 - x1) * (y2 - y1)),
            })

        return detections

    def _publish_results(self, detections: list[dict],
                         header, image: np.ndarray):
        """Publish detection results and alerts."""
        for det in detections:
            msg = DefectDetection()
            msg.header = header
            msg.defect_type = det['label']
            msg.severity = det['severity']
            msg.confidence = det['confidence']
            msg.defect_area = det['area']

            # Crop defect region
            x1, y1, x2, y2 = [int(v) for v in det['bbox']]
            crop = image[y1:y2, x1:x2]
            msg.crop_image = self._bridge.cv2_to_imgmsg(crop, encoding='bgr8')

            self._result_pub.publish(msg)

            # Alert for critical defects
            if det['severity'] == 'critical':
                alert = Alert()
                alert.header = header
                alert.severity = Alert.SEVERITY_CRITICAL if hasattr(Alert, 'SEVERITY_CRITICAL') else 2
                alert.alert_type = 'defect'
                alert.message = (
                    f"CRITICAL defect: {det['label']} "
                    f"(confidence={det['confidence']:.0%}, "
                    f"area={det['area']:.0f}px²)")
                alert.suggested_actions = [
                    'Stop production line immediately',
                    'Quarantine affected batch',
                    'Notify quality supervisor',
                ]
                self._alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = DefectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
