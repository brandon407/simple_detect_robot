"""
DeepSeek Provider — via OpenAI-compatible API.

DeepSeek API is fully OpenAI-compatible, making integration straightforward.
"""
import logging
from typing import Optional

from .base_provider import BaseLLMProvider, LLMResponse, ChatMessage

logger = logging.getLogger(__name__)

try:
    from openai import AsyncOpenAI
    HAS_OPENAI = True
except ImportError:
    HAS_OPENAI = False


class DeepSeekProvider(BaseLLMProvider):
    """DeepSeek API provider using OpenAI-compatible interface.

    API docs: https://platform.deepseek.com/api-docs/
    """

    DEFAULT_BASE = 'https://api.deepseek.com/v1'
    DEFAULT_MODEL = 'deepseek-chat'

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
        """Send chat completion via DeepSeek API."""
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
                provider='deepseek',
                tokens_used=response.usage.total_tokens if response.usage else 0,
                finish_reason=choice.finish_reason or 'stop',
            )
        except Exception as e:
            logger.error(f"DeepSeek API error: {e}")
            return LLMResponse(
                text='', model=self._model, provider='deepseek',
                finish_reason='error', error=str(e))

    async def chat_with_image(self, prompt: str, images: list[bytes],
                               system_prompt: str = '',
                               **kwargs) -> LLMResponse:
        """Multi-modal chat — DeepSeek currently has limited vision support.

        Falls back to describing the number of images provided.
        """
        # DeepSeek doesn't natively support multi-modal yet (as of 2025)
        # We describe the images in text as a workaround
        if not self._client:
            return self._mock_image_chat(prompt, images)

        text_prompt = (
            f"{prompt}\n\n"
            f"[系统提示：此查询附带了{len(images)}张巡检图像。"
            f"请根据问题进行分析，并注明无法直接查看图像。]"
        )

        messages = [ChatMessage(role='user', content=text_prompt)]
        return await self.chat(messages, system_prompt, **kwargs)

    def _build_messages(self, messages: list[ChatMessage],
                        system_prompt: str) -> list[dict]:
        api_msgs = []
        if system_prompt:
            api_msgs.append({'role': 'system', 'content': system_prompt})
        for msg in messages:
            api_msgs.append({'role': msg.role, 'content': msg.content})
        return api_msgs

    def _mock_chat(self, messages: list[ChatMessage],
                   system_prompt: str) -> LLMResponse:
        last_msg = messages[-1].content if messages else ''
        return LLMResponse(
            text=f"[DeepSeek Mock] 针对您的工业巡检问题「{last_msg[:60]}...」\n\n"
                 f"## 分析结果\n"
                 f"根据工业标准和设备运行数据，建议：\n"
                 f"1. 检查设备运行日志确认异常时间点\n"
                 f"2. 对比历史数据判断是否为趋势性问题\n"
                 f"3. 按照预防性维护计划安排检修\n\n"
                 f"## 风险评估\n"
                 f"当前风险等级：中等。建议在下一个维护窗口进行处理。",
            model='deepseek-chat (mock)',
            provider='deepseek',
            tokens_used=0,
        )

    def _mock_image_chat(self, prompt: str, images: list[bytes]) -> LLMResponse:
        return LLMResponse(
            text=f"[DeepSeek Mock] 已接收{len(images)}张巡检图像。\n\n"
                 f"针对「{prompt[:60]}...」的分析：\n"
                 f"- 图像质量：可接受\n"
                 f"- 检测到需关注的区域：2处\n"
                 f"- 建议：人工复核确认",
            model='deepseek-chat (mock)',
            provider='deepseek',
            tokens_used=0,
        )
