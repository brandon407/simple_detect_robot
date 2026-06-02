"""
ONNX Runtime Inference Engine.

Supports CPU/CUDA/TensorRT backends with automatic device selection.
Provides a clean API for industrial inspection model inference.
"""
import logging
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# ONNX Runtime is optional — graceful fallback if not installed
try:
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False
    logger.warning("onnxruntime not installed — using mock inference mode")


@dataclass
class InferenceResult:
    """Standardized inference output."""
    outputs: list[np.ndarray]       # Raw model outputs per output name
    output_names: list[str]         # Names of output tensors
    inference_time_ms: float        # Inference latency in milliseconds
    device: str                     # 'cpu' | 'cuda' | 'tensorrt' | 'mock'


class InferenceEngine:
    """ONNX Runtime inference engine for industrial inspection models."""

    def __init__(self, model_path: str, device: str = 'auto'):
        """
        Args:
            model_path: Path to the ONNX model file.
            device: 'cpu' | 'cuda' | 'tensorrt' | 'auto'.
                    'auto' tries CUDA → CPU.
        """
        self._model_path = model_path
        self._session: Optional[ort.InferenceSession] = None
        self._device = 'mock'
        self._input_name: str = ''
        self._input_shape: tuple = ()
        self._output_names: list[str] = []
        self._warm = False

        if HAS_ONNX and os.path.exists(model_path):
            self._init_onnx(device)
        else:
            logger.info(f"Model not found or ONNX unavailable: {model_path} — using mock mode")
            self._device = 'mock'

    def _init_onnx(self, device: str):
        """Initialize ONNX Runtime session with appropriate execution provider."""
        providers = []

        if device in ('cuda', 'auto'):
            if 'CUDAExecutionProvider' in ort.get_available_providers():
                providers.append(('CUDAExecutionProvider', {
                    'device_id': 0,
                    'arena_extend_strategy': 'kNextPowerOfTwo',
                    'cudnn_conv_algo_search': 'EXHAUSTIVE',
                }))
                self._device = 'cuda'
            elif device == 'cuda':
                logger.warning("CUDA requested but not available — falling back to CPU")

        if device in ('tensorrt', 'auto') and not providers:
            if 'TensorrtExecutionProvider' in ort.get_available_providers():
                providers.append(('TensorrtExecutionProvider', {
                    'device_id': 0,
                    'trt_fp16_enable': True,
                }))
                self._device = 'tensorrt'

        if not providers:
            providers.append('CPUExecutionProvider')
            self._device = 'cpu'

        try:
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = (
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL)
            sess_options.intra_op_num_threads = 4
            sess_options.inter_op_num_threads = 2

            self._session = ort.InferenceSession(
                self._model_path,
                sess_options=sess_options,
                providers=providers,
            )

            # Cache metadata
            self._input_name = self._session.get_inputs()[0].name
            self._input_shape = tuple(self._session.get_inputs()[0].shape)
            self._output_names = [o.name for o in self._session.get_outputs()]

            logger.info(
                f"ONNX model loaded: {os.path.basename(self._model_path)} "
                f"on {self._device} "
                f"(input={self._input_name}{self._input_shape}, "
                f"outputs={self._output_names})")
        except Exception as e:
            logger.error(f"Failed to load ONNX model: {e}")
            self._device = 'mock'
            self._session = None

    @property
    def is_mock(self) -> bool:
        return self._device == 'mock'

    @property
    def device(self) -> str:
        return self._device

    @property
    def input_size(self) -> tuple[int, int]:
        """Return (height, width) expected by the model."""
        if self._input_shape and len(self._input_shape) == 4:
            return (self._input_shape[2], self._input_shape[3])
        return (640, 640)  # Default for YOLO-like models

    def warmup(self, num_runs: int = 3):
        """Warm up the inference engine with dummy inputs."""
        if self._device == 'mock':
            self._warm = True
            return

        dummy = np.random.randn(1, *self._input_shape[1:]).astype(np.float32)
        for i in range(num_runs):
            self._session.run(self._output_names, {self._input_name: dummy})
        self._warm = True
        logger.debug(f"Model warmed up ({num_runs} runs)")

    def infer(self, input_tensor: np.ndarray) -> InferenceResult:
        """Run inference on a preprocessed input tensor.

        Args:
            input_tensor: Preprocessed image tensor with shape (1, C, H, W).

        Returns:
            InferenceResult with outputs and timing.
        """
        if self._device == 'mock':
            return self._mock_infer()

        t0 = time.perf_counter()
        outputs = self._session.run(
            self._output_names, {self._input_name: input_tensor})
        elapsed_ms = (time.perf_counter() - t0) * 1000.0

        return InferenceResult(
            outputs=[np.array(o) for o in outputs],
            output_names=list(self._output_names),
            inference_time_ms=elapsed_ms,
            device=self._device,
        )

    def _mock_infer(self) -> InferenceResult:
        """Generate mock inference results for testing without a model.

        Returns dummy detections that simulate finding defects/meters/safety violations.
        """
        # Simulate a small amount of processing time
        time.sleep(0.005)

        # Mock YOLO-style output: [batch, num_detections, 6] where 6 = [x1,y1,x2,y2,conf,class]
        mock_detections = np.array([[
            [100, 80, 200, 180, 0.85, 0],   # Detection 1
            [300, 150, 420, 280, 0.72, 1],   # Detection 2
        ]], dtype=np.float32)

        return InferenceResult(
            outputs=[mock_detections],
            output_names=['detections'],
            inference_time_ms=5.0,
            device='mock',
        )

    def get_model_info(self) -> dict:
        """Return model metadata for monitoring."""
        return {
            'model_path': self._model_path,
            'device': self._device,
            'input_name': self._input_name,
            'input_shape': list(self._input_shape) if self._input_shape else [],
            'output_names': self._output_names,
            'is_warm': self._warm,
            'has_onnx': HAS_ONNX,
        }
