"""
Inspection Orchestrator — central coordination node for industrial inspection.

Manages detector lifecycle, dispatches inspection tasks, collects results,
publishes unified inspection output, and handles alert aggregation.
"""
import logging
import threading
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inspection_msgs.msg import (
    InspectionResult, Alert, DefectDetection, MeterReading, SafetyCheck)
from inspection_msgs.srv import StartInspection

logger = logging.getLogger(__name__)


class InspectionMode(Enum):
    IDLE = 'idle'
    DEFECT = 'defect'
    METER = 'meter'
    SAFETY = 'safety'
    ALL = 'all'


class InspectionOrchestrator(Node):
    """Central node that coordinates all inspection detectors.

    Services:
        /inspection/start  — start an inspection task
        /inspection/stop   — stop current inspection
        /inspection/status — get current inspection status

    Subscriptions:
        /inspection/defect/result   — from DefectDetector
        /inspection/meter/result    — from MeterReader
        /inspection/safety/result   — from SafetyChecker
        /camera/rgb                 — raw camera feed (for monitoring)

    Publishers:
        /inspection/result          — unified InspectionResult
        /inspection/alert           — aggregated alerts
        /inspection/status_info     — current inspection state
    """

    # Detector result topics
    DETECTOR_TOPICS = {
        'defect': '/inspection/defect/result',
        'meter': '/inspection/meter/result',
        'safety': '/inspection/safety/result',
    }
    DETECTOR_ALERT_TOPICS = {
        'defect': '/inspection/defect/alert',
        'meter': '/inspection/meter/alert',
        'safety': '/inspection/safety/alert',
    }

    def __init__(self):
        super().__init__('inspection_orchestrator')

        # Parameters
        self.declare_parameter('default_mode', 'all')
        self.declare_parameter('inspection_timeout', 30.0)
        self.declare_parameter('auto_load_detectors', True)
        self.declare_parameter('processing_rate', 10.0)

        self._default_mode = self.get_parameter('default_mode').value
        self._timeout = self.get_parameter('inspection_timeout').value

        # Bridge
        self._bridge = CvBridge()

        # State
        self._mode = InspectionMode.IDLE
        self._active = False
        self._lock = threading.Lock()
        self._inspection_start_time = 0.0
        self._total_inspections = 0
        self._detection_counts = {'defect': 0, 'meter': 0, 'safety': 0}
        self._last_alert: Optional[Alert] = None
        self._latest_frame: Optional[Image] = None

        # Services
        self._start_srv = self.create_service(
            Trigger, '/inspection/start',
            self._start_callback, callback_group=ReentrantCallbackGroup())
        self._stop_srv = self.create_service(
            Trigger, '/inspection/stop',
            self._stop_callback, callback_group=ReentrantCallbackGroup())
        self._status_srv = self.create_service(
            Trigger, '/inspection/status',
            self._status_callback, callback_group=ReentrantCallbackGroup())

        # Publishers
        self._result_pub = self.create_publisher(
            InspectionResult, '/inspection/result', 10)
        self._alert_pub = self.create_publisher(
            Alert, '/inspection/alert', 10)

        # Subscribers — detector results (type-specific)
        self._defect_sub = self.create_subscription(
            DefectDetection, self.DETECTOR_TOPICS['defect'],
            lambda msg: self._on_detector_result('defect', msg),
            10, callback_group=ReentrantCallbackGroup())
        self._meter_sub = self.create_subscription(
            MeterReading, self.DETECTOR_TOPICS['meter'],
            lambda msg: self._on_detector_result('meter', msg),
            10, callback_group=ReentrantCallbackGroup())
        self._safety_sub = self.create_subscription(
            SafetyCheck, self.DETECTOR_TOPICS['safety'],
            lambda msg: self._on_detector_result('safety', msg),
            10, callback_group=ReentrantCallbackGroup())

        # Subscribers — detector alerts
        for dtype, topic in self.DETECTOR_ALERT_TOPICS.items():
            self.create_subscription(
                Alert, topic,
                lambda msg, t=dtype: self._on_detector_alert(t, msg),
                10, callback_group=ReentrantCallbackGroup())

        # Camera monitoring (low-rate, for status)
        self._camera_sub = self.create_subscription(
            Image, '/camera/rgb', self._camera_callback, 10,
            callback_group=ReentrantCallbackGroup())

        # Status timer
        self._status_timer = self.create_timer(
            5.0, self._publish_status, callback_group=ReentrantCallbackGroup())

        # Timeout monitor
        self._timeout_timer = self.create_timer(
            1.0, self._check_timeout, callback_group=ReentrantCallbackGroup())

        self.get_logger().info('InspectionOrchestrator ready')

    # ── Service Callbacks ───────────────────────────────────────

    def _start_callback(self, request, response):
        """Start an inspection task."""
        mode_str = request.mode if hasattr(request, 'mode') else self._default_mode

        try:
            mode = InspectionMode(mode_str)
        except ValueError:
            response.success = False
            response.message = f'Invalid mode: {mode_str}. Use: defect, meter, safety, all'
            return response

        started = self.start_inspection(mode)
        response.success = started
        response.message = (
            f'Inspection started (mode={mode.value})'
            if started else 'Failed to start inspection')
        return response

    def _stop_callback(self, request, response):
        """Stop the current inspection."""
        self.stop_inspection()
        response.success = True
        response.message = 'Inspection stopped'
        return response

    def _status_callback(self, request, response):
        """Return current inspection status."""
        status = self.get_status()
        response.success = True
        response.message = (
            f'Mode: {status["mode"]}, Active: {status["active"]}, '
            f'Inspections: {status["total_inspections"]}, '
            f'Detections: {status["detection_counts"]}'
        )
        return response

    # ── Public API ──────────────────────────────────────────────

    def start_inspection(self, mode: InspectionMode = InspectionMode.ALL) -> bool:
        """Start an inspection task.

        Args:
            mode: Which detector(s) to activate.

        Returns:
            True if started successfully.
        """
        with self._lock:
            if self._active:
                self.get_logger().warn('Inspection already active')
                return False

            self._mode = mode
            self._active = True
            self._inspection_start_time = time.time()
            self._detection_counts = {'defect': 0, 'meter': 0, 'safety': 0}

            self.get_logger().info(f'Inspection started: mode={mode.value}')
            self._total_inspections += 1
            return True

    def stop_inspection(self):
        """Stop the current inspection task."""
        with self._lock:
            if not self._active:
                return
            self._active = False
            elapsed = time.time() - self._inspection_start_time
            self.get_logger().info(
                f'Inspection stopped after {elapsed:.1f}s '
                f'(detections: {self._detection_counts})')
            self._mode = InspectionMode.IDLE

    def get_status(self) -> dict:
        """Return current inspection status."""
        with self._lock:
            return {
                'mode': self._mode.value,
                'active': self._active,
                'elapsed': time.time() - self._inspection_start_time if self._active else 0,
                'total_inspections': self._total_inspections,
                'detection_counts': dict(self._detection_counts),
                'latest_alert': (
                    self._last_alert.message if self._last_alert else None),
            }

    def is_active(self) -> bool:
        return self._active

    def is_mode_active(self, detector_type: str) -> bool:
        """Check if a specific detector type is currently active."""
        if not self._active:
            return False
        if self._mode == InspectionMode.ALL:
            return True
        return self._mode.value == detector_type

    # ── Event Handlers ──────────────────────────────────────────

    def _on_detector_result(self, detector_type: str, msg):
        """Handle incoming detection result from a detector."""
        if not self.is_mode_active(detector_type):
            return

        with self._lock:
            self._detection_counts[detector_type] = (
                self._detection_counts.get(detector_type, 0) + 1)

        # Forward as unified result
        result = InspectionResult()
        result.header = msg.header
        result.detector_type = detector_type
        result.status = 'ok'
        result.description = f'{detector_type} detection completed'
        result.confidence = float(getattr(msg, 'confidence', 0.0))
        result.detection_pose = getattr(msg, 'detection_pose', None)

        self._result_pub.publish(result)

    def _on_detector_alert(self, detector_type: str, msg: Alert):
        """Handle incoming alert from a detector."""
        if not self.is_mode_active(detector_type):
            return

        with self._lock:
            self._last_alert = msg

        self.get_logger().warn(
            f'[{detector_type}] ALERT (severity={msg.severity}): {msg.message}')

        # Forward alert (aggregated)
        self._alert_pub.publish(msg)

    def _camera_callback(self, msg: Image):
        """Store latest camera frame for monitoring."""
        self._latest_frame = msg

    # ── Periodic Tasks ──────────────────────────────────────────

    def _publish_status(self):
        """Periodically log inspection status."""
        if not self._active:
            return
        status = self.get_status()
        self.get_logger().debug(
            f'Inspection status: mode={status["mode"]}, '
            f'elapsed={status["elapsed"]:.0f}s, '
            f'detections={status["detection_counts"]}')

    def _check_timeout(self):
        """Check if the current inspection has timed out."""
        with self._lock:
            if not self._active:
                return
            if self._timeout <= 0:
                return
            elapsed = time.time() - self._inspection_start_time
            if elapsed > self._timeout:
                self.get_logger().warn(f'Inspection timed out after {elapsed:.1f}s')
                self._active = False
                self._mode = InspectionMode.IDLE


def main(args=None):
    rclpy.init(args=args)
    node = InspectionOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
