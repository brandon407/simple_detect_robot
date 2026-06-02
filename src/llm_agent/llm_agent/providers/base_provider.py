"""
Abstract base class for LLM API providers.

All providers must implement the async chat and chat_with_image methods.
Supports OpenAI-compatible and custom API formats.
"""
import base64
import io
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class LLMResponse:
    """Standardized LLM response across providers."""
    text: str
    model: str = ''
    provider: str = ''
    tokens_used: int = 0
    finish_reason: str = 'stop'  # 'stop' | 'length' | 'error'
    error: Optional[str] = None

    @property
    def ok(self) -> bool:
        return self.error is None and self.finish_reason != 'error'


@dataclass
class ChatMessage:
    """A single chat message."""
    role: str  # 'system' | 'user' | 'assistant'
    content: str
    images: list[bytes] = field(default_factory=list)  # Raw image bytes for multi-modal


class BaseLLMProvider(ABC):
    """Abstract interface for LLM API providers."""

    def __init__(self, model: str, api_key: str = '', api_base: str = '',
                 max_tokens: int = 4096, temperature: float = 0.1):
        self._model = model
        self._api_key = api_key
        self._api_base = api_base
        self._max_tokens = max_tokens
        self._temperature = temperature

    @property
    def model(self) -> str:
        return self._model

    @property
    def provider_name(self) -> str:
        return self.__class__.__name__.replace('Provider', '').lower()

    def is_configured(self) -> bool:
        """Check if the provider has valid API credentials."""
        return bool(self._api_key)

    @abstractmethod
    async def chat(self, messages: list[ChatMessage],
                   system_prompt: str = '',
                   **kwargs) -> LLMResponse:
        """Send a chat completion request.

        Args:
            messages: List of chat messages.
            system_prompt: Optional system-level instruction.
            **kwargs: Provider-specific parameters.

        Returns:
            LLMResponse with the model's reply.
        """
        ...

    @abstractmethod
    async def chat_with_image(self,
                               prompt: str,
                               images: list[bytes],
                               system_prompt: str = '',
                               **kwargs) -> LLMResponse:
        """Send a multi-modal request with images.

        Args:
            prompt: User's question about the images.
            images: List of raw image bytes (PNG/JPEG).
            system_prompt: Optional system instruction.
            **kwargs: Provider-specific parameters.

        Returns:
            LLMResponse with analysis.
        """
        ...

    def get_model_info(self) -> dict:
        """Return model metadata."""
        return {
            'provider': self.provider_name,
            'model': self._model,
            'api_base': self._api_base[:50] + '...' if self._api_base else 'none',
            'configured': self.is_configured(),
            'max_tokens': self._max_tokens,
        }

    @staticmethod
    def _encode_image(image_bytes: bytes) -> str:
        """Encode image bytes to base64 data URI."""
        b64 = base64.b64encode(image_bytes).decode('utf-8')
        return f'data:image/jpeg;base64,{b64}'

    @staticmethod
    def _image_to_pil_bytes(image_bytes: bytes, max_size: tuple = (1024, 1024)) -> bytes:
        """Resize image to fit within max_size while preserving aspect ratio."""
        try:
            from PIL import Image
            import io as io_module
            img = Image.open(io_module.BytesIO(image_bytes))
            img.thumbnail(max_size, Image.LANCZOS)
            buf = io_module.BytesIO()
            img.save(buf, format='JPEG', quality=85)
            return buf.getvalue()
        except ImportError:
            return image_bytes
