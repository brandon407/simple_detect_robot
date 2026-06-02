"""Abstract base class for LLM API providers."""
from abc import ABC, abstractmethod


class BaseLLMProvider(ABC):
    """Abstract provider interface for large language model APIs."""

    @abstractmethod
    async def chat(self, messages: list[dict], **kwargs) -> str:
        """Send a chat completion request."""
        pass

    @abstractmethod
    async def chat_with_image(self, messages: list[dict], images: list[bytes]) -> str:
        """Send a multi-modal chat request with images."""
        pass

    @abstractmethod
    def get_model_info(self) -> dict:
        """Return model information."""
        pass
