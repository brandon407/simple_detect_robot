"""Base detector abstract class - plugin interface for inspection detectors."""
from abc import ABC, abstractmethod
import numpy as np


class BaseDetector(ABC):
    """Abstract base class for all inspection detectors."""

    @abstractmethod
    def initialize(self, config: dict) -> bool:
        """Initialize detector with configuration."""
        pass

    @abstractmethod
    def detect(self, image: np.ndarray, depth: np.ndarray = None) -> dict:
        """Run detection on an image. Returns detection result dict."""
        pass

    @abstractmethod
    def get_type(self) -> str:
        """Return detector type: 'defect' | 'meter' | 'safety'."""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Clean up resources."""
        pass
