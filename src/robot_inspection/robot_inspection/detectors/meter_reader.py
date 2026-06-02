"""
Meter Reader — instrument gauge/digital display reading recognition.

Detects meter panels, reads analog gauge values (needle position) and
digital displays via OCR. Compares readings against normal ranges.
"""
import logging
import math
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inspection_msgs.msg import MeterReading, Alert

logger = logging.getLogger(__name__)

# Try to import PaddleOCR — fall back gracefully
try:
    from paddleocr import PaddleOCR
    HAS_OCR = True
except ImportError:
    HAS_OCR = False
    logger.warning("PaddleOCR not installed — using template-based reading only")


class MeterReaderNode(Node):
    """ROS2 node for instrument meter reading recognition."""

    def __init__(self):
        super().__init__('meter_reader')

        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('processing_rate', 2.0)  # Hz — meters change slowly
        self.declare_parameter('ocr_lang', 'en')
        self.declare_parameter('enable_mock', True)
        self.declare_parameter('meter_templates_dir', '')

        model_path = self.get_parameter('model_path').value
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        device = self.get_parameter('device').value
        rate = self.get_parameter('processing_rate').value
        ocr_lang = self.get_parameter('ocr_lang').value
        enable_mock = self.get_parameter('enable_mock').value

        # Inference engine (for meter detection, not reading)
        from ..models.inference_engine import InferenceEngine
        from ..utils.preprocessor import ImagePreprocessor
        if not model_path and enable_mock:
            model_path = '__mock__'
        self._engine = InferenceEngine(model_path, device=device)
        self._preprocessor = ImagePreprocessor(target_size=self._engine.input_size)
        self._bridge = CvBridge()

        # OCR engine (optional)
        self._ocr: Optional['PaddleOCR'] = None
        if HAS_OCR:
            try:
                self._ocr = PaddleOCR(lang=ocr_lang, use_angle_cls=True, show_log=False)
                self.get_logger().info(f'PaddleOCR initialized (lang={ocr_lang})')
            except Exception as e:
                self.get_logger().warn(f'PaddleOCR init failed: {e}')

        # State
        self._last_process_time = 0.0
        self._process_interval = 1.0 / rate if rate > 0 else 0.0

        # Publishers
        self._result_pub = self.create_publisher(
            MeterReading, '/inspection/meter/result', 10)
        self._alert_pub = self.create_publisher(
            Alert, '/inspection/meter/alert', 10)
        self._annotated_pub = self.create_publisher(
            Image, '/inspection/meter/annotated', 10)

        # Subscriber
        self._image_sub = self.create_subscription(
            Image, '/camera/rgb', self._image_callback, 10,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'MeterReader ready (device={self._engine.device}, '
            f'rate={rate}Hz, ocr={self._ocr is not None}, mock={self._engine.is_mock})')

    def _image_callback(self, msg: Image):
        """Process camera images for meter reading."""
        now = time.time()
        if now - self._last_process_time < self._process_interval:
            return
        self._last_process_time = now

        try:
            image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self._engine.is_mock:
                result = self._mock_read(image)
            else:
                result = self._read_meter(image)

            if result:
                self._publish_result(result, msg.header, image)

        except Exception as e:
            self.get_logger().error(f'Meter reading error: {e}', throttle_duration_sec=5.0)

    def _read_meter(self, image: np.ndarray) -> Optional[dict]:
        """Full meter reading pipeline: detect → read → validate."""
        h, w = image.shape[:2]

        # Step 1: Detect meter region (using detection model)
        tensor, scale, (pad_x, pad_y) = self._preprocessor.preprocess(image)
        result = self._engine.infer(tensor)

        # Parse meter detection bbox
        meter_bbox = None
        if len(result.outputs) > 0 and len(result.outputs[0]) > 0:
            dets = result.outputs[0][0]
            if len(dets) >= 6:
                x1, y1, x2, y2 = dets[:4]
                x1 = max(0, int((x1 - pad_x) / scale))
                y1 = max(0, int((y1 - pad_y) / scale))
                x2 = min(w, int((x2 - pad_x) / scale))
                y2 = min(h, int((y2 - pad_y) / scale))
                meter_bbox = (x1, y1, x2, y2)

        if meter_bbox is None:
            # Assume whole image is the meter
            meter_roi = image
            meter_bbox = (0, 0, w, h)
        else:
            meter_roi = image[meter_bbox[1]:meter_bbox[3], meter_bbox[0]:meter_bbox[2]]

        # Step 2: Read the value from the ROI
        reading = self._extract_reading(meter_roi)
        if reading is None:
            return None

        return {
            'reading_value': reading['value'],
            'reading_unit': reading.get('unit', ''),
            'meter_type': reading.get('type', 'analog_gauge'),
            'confidence': reading.get('confidence', 0.7),
            'bbox': list(meter_bbox),
            'is_anomaly': self._check_anomaly(reading['value']),
        }

    def _extract_reading(self, roi: np.ndarray) -> Optional[dict]:
        """Extract numeric reading from a meter ROI.

        Strategy:
        1. Try OCR for digital displays
        2. Fall back to analog gauge needle detection
        """
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Try OCR if available
        if self._ocr is not None:
            try:
                ocr_result = self._ocr.ocr(roi, cls=True)
                if ocr_result and ocr_result[0]:
                    texts = [line[1][0] for line in ocr_result[0] if line[1][1] > 0.5]
                    for text in texts:
                        value = self._parse_numeric(text)
                        if value is not None:
                            return {
                                'value': value,
                                'type': 'digital_display',
                                'confidence': 0.85,
                                'raw_text': text,
                            }
            except Exception as e:
                logger.debug(f"OCR failed: {e}")

        # Fall back: analog gauge needle detection
        return self._detect_needle(gray)

    def _detect_needle(self, gray: np.ndarray) -> Optional[dict]:
        """Detect analog gauge needle angle and convert to reading.

        Uses Hough Line Transform to find the needle.
        """
        h, w = gray.shape
        cx, cy = w // 2, h // 2

        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=min(w, h) // 2)

        if lines is None:
            return None

        # Find the longest line closest to center
        best_line = None
        best_score = -1
        for line in lines:
            rho, theta = line[0]
            a, b = math.cos(theta), math.sin(theta)
            x0, y0 = a * rho, b * rho

            # Distance from line to center
            dist_to_center = abs((a * cx + b * cy) - rho)
            score = rho - dist_to_center * 2  # Favor long lines near center

            if score > best_score:
                best_score = score
                best_line = (rho, theta)

        if best_line is None:
            return None

        _, theta = best_line
        needle_angle = math.degrees(theta)

        # Convert angle to gauge reading (simplified: 0-180° → 0-100%)
        # Real implementation needs calibration per gauge type
        normalized = (needle_angle + 90) % 360
        if normalized > 180:
            normalized = 360 - normalized
        value = (normalized / 180.0) * 100.0

        return {
            'value': round(value, 1),
            'type': 'analog_gauge',
            'confidence': min(best_score / 200.0, 0.95),
            'needle_angle': needle_angle,
        }

    def _mock_read(self, image: np.ndarray) -> dict:
        """Generate mock meter reading for simulation testing.

        Reads are randomized but within plausible industrial ranges.
        """
        import random
        random.seed(int(time.time() * 1000) % 10000)

        h, w = image.shape[:2]

        # Simulate different meter types
        meter_types = [
            {'type': 'analog_gauge', 'value': random.uniform(0.2, 0.8), 'unit': 'MPa',
             'min': 0.0, 'max': 1.0},
            {'type': 'digital_display', 'value': random.uniform(20, 80), 'unit': '°C',
             'min': 10.0, 'max': 90.0},
            {'type': 'level_indicator', 'value': random.uniform(0.3, 0.9), 'unit': 'm',
             'min': 0.0, 'max': 1.5},
        ]
        meter = random.choice(meter_types)

        is_anomaly = meter['value'] < meter['min'] or meter['value'] > meter['max']

        return {
            'reading_value': round(meter['value'], 2),
            'reading_unit': meter['unit'],
            'meter_type': meter['type'],
            'confidence': random.uniform(0.7, 0.95),
            'bbox': [int(w * 0.1), int(h * 0.15), int(w * 0.9), int(h * 0.85)],
            'is_anomaly': is_anomaly,
        }

    def _parse_numeric(self, text: str) -> Optional[float]:
        """Parse a numeric value from OCR text, handling common formats."""
        import re
        # Remove common OCR artifacts
        text = text.replace('O', '0').replace('l', '1').replace(' ', '')
        match = re.search(r'[-+]?\d*\.?\d+', text)
        if match:
            return float(match.group())
        return None

    def _check_anomaly(self, value: float,
                       min_normal: float = 0.0,
                       max_normal: float = 100.0) -> bool:
        """Check if a reading is outside the normal range."""
        return value < min_normal or value > max_normal

    def _publish_result(self, reading: dict, header, image: np.ndarray):
        """Publish meter reading result and alert if anomalous."""
        msg = MeterReading()
        msg.header = header
        msg.meter_type = reading['meter_type']
        msg.reading_value = float(reading['reading_value'])
        msg.reading_unit = reading.get('reading_unit', '')
        msg.confidence = float(reading.get('confidence', 0.8))
        msg.min_normal = 0.0
        msg.max_normal = 100.0
        msg.is_anomaly = reading.get('is_anomaly', False)

        # Add meter ROI image
        x1, y1, x2, y2 = [int(v) for v in reading['bbox']]
        roi = image[y1:y2, x1:x2]
        msg.meter_image = self._bridge.cv2_to_imgmsg(roi, encoding='bgr8')

        self._result_pub.publish(msg)

        if msg.is_anomaly:
            alert = Alert()
            alert.header = header
            alert.severity = 1  # warning
            alert.alert_type = 'meter'
            alert.message = (
                f"ANOMALOUS reading: {msg.reading_value}{msg.reading_unit} "
                f"on {msg.meter_type}")
            alert.suggested_actions = [
                'Verify reading manually',
                'Check equipment for malfunction',
                'Log reading for trend analysis',
            ]
            self._alert_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = MeterReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
