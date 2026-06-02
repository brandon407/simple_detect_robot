"""
Model Manager — handles lifecycle of multiple ONNX models.

Features: lazy loading, hot-reload, memory monitoring, model registry.
"""
import logging
import os
import threading
from typing import Optional

import yaml

from .inference_engine import InferenceEngine

logger = logging.getLogger(__name__)


class ModelManager:
    """Manage multiple inference models with lifecycle control."""

    def __init__(self, config_path: Optional[str] = None):
        self._models: dict[str, InferenceEngine] = {}
        self._config: dict = {}
        self._lock = threading.Lock()
        self._model_dir = os.path.expanduser('~/.inspection_robot/models')

        if config_path and os.path.exists(config_path):
            self.load_config(config_path)

    def load_config(self, config_path: str):
        """Load detector configuration from YAML."""
        with open(config_path, 'r') as f:
            self._config = yaml.safe_load(f)
        logger.info(f"Loaded detector config: {config_path}")

    def get_model(self, detector_type: str,
                  model_path: Optional[str] = None,
                  device: str = 'auto') -> InferenceEngine:
        """Get or create an inference engine for a detector type.

        Args:
            detector_type: 'defect' | 'meter' | 'safety'.
            model_path: Path to ONNX model. Uses config if None.
            device: Device preference.

        Returns:
            InferenceEngine instance.
        """
        with self._lock:
            if detector_type in self._models:
                return self._models[detector_type]

            # Resolve model path
            if model_path is None:
                model_path = self._resolve_path(detector_type)

            engine = InferenceEngine(model_path, device=device)
            self._models[detector_type] = engine
            return engine

    def reload_model(self, detector_type: str,
                     model_path: Optional[str] = None,
                     device: str = 'auto'):
        """Hot-reload a model at runtime."""
        with self._lock:
            if detector_type in self._models:
                self._models[detector_type] = None
            return self.get_model(detector_type, model_path, device)

    def unload_model(self, detector_type: str):
        """Unload a model to free memory."""
        with self._lock:
            if detector_type in self._models:
                self._models.pop(detector_type)
                logger.info(f"Unloaded model: {detector_type}")

    def warmup_all(self):
        """Warm up all loaded models."""
        for name, engine in self._models.items():
            if not engine.is_mock:
                engine.warmup()

    def get_status(self) -> dict:
        """Return status of all models."""
        return {
            name: engine.get_model_info()
            for name, engine in self._models.items()
        }

    def _resolve_path(self, detector_type: str) -> str:
        """Resolve model path from config or defaults."""
        # Try config first
        detector_config = self._config.get(f'{detector_type}_detector', {})
        path = detector_config.get('model_path', '')

        if path and os.path.exists(path):
            return path

        # Try model directory
        default_names = {
            'defect': 'yolov8n_defect.onnx',
            'meter': 'meter_detection.onnx',
            'safety': 'yolov8n_safety.onnx',
        }
        default_path = os.path.join(
            self._model_dir, default_names.get(detector_type, 'model.onnx'))

        return default_path  # May not exist → mock mode

    @property
    def model_dir(self) -> str:
        return self._model_dir
