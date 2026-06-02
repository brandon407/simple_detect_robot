"""
Image Preprocessor — prepares images for inference models.

Handles: resize, normalization, color conversion, batching.
"""
import logging
from typing import Optional

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class ImagePreprocessor:
    """Image preprocessing pipeline for industrial inspection models."""

    # Default ImageNet normalization
    DEFAULT_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    DEFAULT_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)

    def __init__(self,
                 target_size: tuple[int, int] = (640, 640),
                 normalize: bool = True,
                 mean: Optional[np.ndarray] = None,
                 std: Optional[np.ndarray] = None,
                 color_order: str = 'RGB'):
        """
        Args:
            target_size: (height, width) for model input.
            normalize: Apply mean/std normalization.
            mean: Normalization mean per channel.
            std: Normalization std per channel.
            color_order: 'RGB' or 'BGR' for output.
        """
        self._target_size = target_size
        self._normalize = normalize
        self._mean = mean if mean is not None else self.DEFAULT_MEAN
        self._std = std if std is not None else self.DEFAULT_STD
        self._color_order = color_order

    def preprocess(self, image: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        """Preprocess an image for model inference.

        Args:
            image: Input image in BGR format (OpenCV default), shape (H, W, 3).

        Returns:
            (input_tensor, scale_ratio, padding) where:
            - input_tensor: (1, 3, H, W) float32 normalized tensor
            - scale_ratio: scaling factor applied
            - padding: (pad_x, pad_y) applied
        """
        h, w = image.shape[:2]
        th, tw = self._target_size

        # Resize maintaining aspect ratio with letterbox padding
        scale = min(tw / w, th / h)
        new_w, new_h = int(w * scale), int(h * scale)
        pad_x = (tw - new_w) / 2
        pad_y = (th - new_h) / 2

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # Letterbox
        letterbox = np.full((th, tw, 3), 114, dtype=np.uint8)
        letterbox[int(pad_y):int(pad_y) + new_h, int(pad_x):int(pad_x) + new_w] = resized

        # Color conversion
        if self._color_order == 'RGB':
            letterbox = cv2.cvtColor(letterbox, cv2.COLOR_BGR2RGB)

        # Normalize
        tensor = letterbox.astype(np.float32) / 255.0
        if self._normalize:
            tensor = (tensor - self._mean) / self._std

        # HWC → CHW → BCHW
        tensor = np.transpose(tensor, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0).astype(np.float32)

        return tensor, scale, (pad_x, pad_y)

    def preprocess_batch(self, images: list[np.ndarray]) -> np.ndarray:
        """Preprocess a batch of images into a single tensor.

        Args:
            images: List of BGR images, all expected to be the same size.

        Returns:
            Batched tensor of shape (N, 3, H, W).
        """
        tensors = []
        for img in images:
            t, _, _ = self.preprocess(img)
            tensors.append(t[0])  # Remove batch dim
        return np.stack(tensors, axis=0).astype(np.float32)

    @staticmethod
    def denormalize(tensor: np.ndarray) -> np.ndarray:
        """Convert normalized tensor back to displayable image."""
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        img = tensor.copy()
        if img.ndim == 4:
            img = img[0]
        img = np.transpose(img, (1, 2, 0))
        img = img * std + mean
        img = np.clip(img * 255, 0, 255).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    @staticmethod
    def enhance_for_inspection(image: np.ndarray) -> np.ndarray:
        """Apply industrial inspection image enhancements.

        - CLAHE for better contrast (helps detect subtle defects)
        - Slight sharpening
        """
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)

        # Light sharpening
        kernel = np.array([[-1, -1, -1],
                           [-1,  9, -1],
                           [-1, -1, -1]]) / 1.0
        enhanced = cv2.filter2D(enhanced, -1, kernel * 0.3 + np.eye(3)[:1] * 0.7)

        return enhanced
