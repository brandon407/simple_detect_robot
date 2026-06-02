"""
Qwen (通义千问) Provider — via DashScope OpenAI-compatible API.

Supports: text chat, multi-modal (Qwen-VL for image understanding).
"""
import json
import logging
from typing import Optional

from .base_provider import BaseLLMProvider, LLMResponse, ChatMessage

logger = logging.getLogger(__name__)

# Try importing OpenAI SDK (used by both Qwen and DeepSeek)
try:
    from openai import AsyncOpenAI
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False
    logger.warning("openai package not installed — Qwen will use mock mode")


class QwenProvider(BaseLLMProvider):
    """通义千问 API provider using OpenAI-compatible interface.

    API docs: https://help.aliyun.com/zh/dashscope/
    """

    DEFAULT_BASE = 'https://dashscope.aliyuncs.com/compatible-mode/v1'
    DEFAULT_MODEL = 'qwen-plus'

    def __init__(self, api_key: str = '', api_base: str = '',
                 model: str = '', max_tokens: int = 4096):
        super().__init__(
            model=model or self.DEFAULT_MODEL,
            api_key=api_key,
            api_base=api_base or self.DEFAULT_BASE,
            max_tokens=max_tokens,
        )
        self._client: Optional['AsyncOpenAI'] = None
        if HAS_OPENAI and api_key:
            self._client = AsyncOpenAI(
                api_key=api_key,
                base_url=self._api_base,
            )

    def is_configured(self) -> bool:
        return HAS_OPENAI and bool(self._api_key)

    async def chat(self, messages: list[ChatMessage],
                   system_prompt: str = '',
                   **kwargs) -> LLMResponse:
        """Send chat completion via Qwen API."""
        if not self._client:
            return self._mock_chat(messages, system_prompt)

        try:
            api_messages = self._build_messages(messages, system_prompt)
            response = await self._client.chat.completions.create(
                model=self._model,
                messages=api_messages,
                max_tokens=kwargs.get('max_tokens', self._max_tokens),
                temperature=kwargs.get('temperature', self._temperature),
            )
            choice = response.choices[0]
            return LLMResponse(
                text=choice.message.content or '',
                model=self._model,
                provider='qwen',
                tokens_used=response.usage.total_tokens if response.usage else 0,
                finish_reason=choice.finish_reason or 'stop',
            )
        except Exception as e:
            logger.error(f"Qwen API error: {e}")
            return LLMResponse(
                text='', model=self._model, provider='qwen',
                finish_reason='error', error=str(e))

    async def chat_with_image(self, prompt: str, images: list[bytes],
                               system_prompt: str = '',
                               **kwargs) -> LLMResponse:
        """Multi-modal chat with Qwen-VL."""
        if not self._client:
            return self._mock_image_chat(prompt, images)

        try:
            # Build multi-modal content
            content = []
            for img_bytes in images:
                b64 = self._encode_image(img_bytes)
                content.append({
                    'type': 'image_url',
                    'image_url': {'url': b64},
                })
            content.append({'type': 'text', 'text': prompt})

            api_messages = []
            if system_prompt:
                api_messages.append({'role': 'system', 'content': system_prompt})
            api_messages.append({'role': 'user', 'content': content})

            response = await self._client.chat.completions.create(
                model='qwen-vl-plus',  # Use vision model
                messages=api_messages,
                max_tokens=kwargs.get('max_tokens', self._max_tokens),
            )
            choice = response.choices[0]
            return LLMResponse(
                text=choice.message.content or '',
                model='qwen-vl-plus',
                provider='qwen',
                tokens_used=response.usage.total_tokens if response.usage else 0,
                finish_reason=choice.finish_reason or 'stop',
            )
        except Exception as e:
            logger.error(f"Qwen VL API error: {e}")
            return LLMResponse(
                text='', model='qwen-vl-plus', provider='qwen',
                finish_reason='error', error=str(e))

    def _build_messages(self, messages: list[ChatMessage],
                        system_prompt: str) -> list[dict]:
        """Convert ChatMessage list to OpenAI API format."""
        api_msgs = []
        if system_prompt:
            api_msgs.append({'role': 'system', 'content': system_prompt})
        for msg in messages:
            api_msgs.append({'role': msg.role, 'content': msg.content})
        return api_msgs

    def _mock_chat(self, messages: list[ChatMessage],
                   system_prompt: str) -> LLMResponse:
        """Mock response for testing without API key."""
        last_msg = messages[-1].content if messages else ''
        return LLMResponse(
            text=f"[Qwen Mock] 针对您的问题「{last_msg[:50]}...」，"
                 f"根据工业巡检知识库，建议进行以下步骤：\n"
                 f"1. 确认设备运行参数是否在正常范围内\n"
                 f"2. 检查相关仪表读数是否存在异常\n"
                 f"3. 如发现异常，请参考SOP手册第3章节进行处理",
            model='qwen-plus (mock)',
            provider='qwen',
            tokens_used=0,
        )

    def _mock_image_chat(self, prompt: str, images: list[bytes]) -> LLMResponse:
        """Mock multi-modal response."""
        return LLMResponse(
            text=f"[Qwen-VL Mock] 已分析{len(images)}张图像。"
                 f"针对「{prompt[:50]}...」的分析结果：\n"
                 f"- 图像中检测到1处产品表面划痕（轻微）\n"
                 f"- 仪表读数显示0.65 MPa，处于正常范围\n"
                 f"- 安全区域内人员已正确佩戴安全帽",
            model='qwen-vl-plus (mock)',
            provider='qwen',
            tokens_used=0,
        )
