"""
Ernie (文心一言) Provider — via 百度千帆 API.

Uses Qianfan SDK with OAuth2 authentication (API Key + Secret Key).
"""
import logging
from typing import Optional

from .base_provider import BaseLLMProvider, LLMResponse, ChatMessage

logger = logging.getLogger(__name__)

try:
    import qianfan
    HAS_QIANFAN = True
except ImportError:
    HAS_QIANFAN = False
    logger.warning("qianfan package not installed — Ernie will use mock mode")


class ErnieProvider(BaseLLMProvider):
    """百度文心一言 API provider using 千帆 SDK.

    API docs: https://cloud.baidu.com/doc/WENXINWORKSHOP/
    """

    DEFAULT_MODEL = 'ernie-4.0-8k'

    def __init__(self, api_key: str = '', api_base: str = '',
                 model: str = '', secret_key: str = '',
                 max_tokens: int = 4096):
        super().__init__(
            model=model or self.DEFAULT_MODEL,
            api_key=api_key,
            api_base=api_base,
            max_tokens=max_tokens,
        )
        self._secret_key = secret_key
        self._chat_model: Optional['qianfan.ChatCompletion'] = None

        if HAS_QIANFAN and api_key and secret_key:
            try:
                import qianfan
                self._chat_model = qianfan.ChatCompletion(
                    model=self._model,
                    ak=api_key,
                    sk=secret_key,
                )
            except Exception as e:
                logger.error(f"Qianfan init error: {e}")

    def is_configured(self) -> bool:
        return HAS_QIANFAN and bool(self._api_key) and bool(self._secret_key)

    async def chat(self, messages: list[ChatMessage],
                   system_prompt: str = '',
                   **kwargs) -> LLMResponse:
        """Send chat completion via Ernie API."""
        if not self._chat_model:
            return self._mock_chat(messages, system_prompt)

        try:
            # Build messages for Qianfan format
            api_messages = self._build_messages(messages, system_prompt)

            # Run in thread since qianfan is synchronous
            import asyncio
            loop = asyncio.get_event_loop()
            response = await loop.run_in_executor(
                None,
                lambda: self._chat_model.do(
                    messages=api_messages,
                    max_output_tokens=kwargs.get('max_tokens', self._max_tokens),
                    temperature=kwargs.get('temperature', self._temperature),
                )
            )

            return LLMResponse(
                text=response.get('result', ''),
                model=self._model,
                provider='ernie',
                tokens_used=response.get('usage', {}).get('total_tokens', 0),
                finish_reason='stop',
            )
        except Exception as e:
            logger.error(f"Ernie API error: {e}")
            return LLMResponse(
                text='', model=self._model, provider='ernie',
                finish_reason='error', error=str(e))

    async def chat_with_image(self, prompt: str, images: list[bytes],
                               system_prompt: str = '',
                               **kwargs) -> LLMResponse:
        """Multi-modal chat — Ernie 4.0 supports vision via specific endpoint."""
        if not self._chat_model:
            return self._mock_image_chat(prompt, images)

        # For now, Ernie vision support requires a different model endpoint
        # We provide a fallback with image count context
        text_prompt = (
            f"{prompt}\n\n"
            f"[附带了{len(images)}张巡检现场图像。请基于文字描述进行分析。]"
        )
        messages = [ChatMessage(role='user', content=text_prompt)]
        return await self.chat(messages, system_prompt, **kwargs)

    def _build_messages(self, messages: list[ChatMessage],
                        system_prompt: str) -> list[dict]:
        """Convert ChatMessage list to Qianfan API format."""
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
            text=f"[Ernie Mock] 关于「{last_msg[:60]}...」的工业巡检建议：\n\n"
                 f"## 诊断分析\n"
                 f"根据GB/T工业标准和设备运行参数，初步判断：\n"
                 f"- 设备运行状态：正常范围内波动\n"
                 f"- 建议关注指标：压力、温度、振动\n\n"
                 f"## 处理建议\n"
                 f"1. 加强巡检频率至每4小时一次\n"
                 f"2. 准备备品备件以防突发故障\n"
                 f"3. 48小时内安排专业工程师现场确认",
            model='ernie-4.0-8k (mock)',
            provider='ernie',
            tokens_used=0,
        )

    def _mock_image_chat(self, prompt: str, images: list[bytes]) -> LLMResponse:
        return LLMResponse(
            text=f"[Ernie Mock] 已完成{len(images)}张巡检图像的多模态分析。\n\n"
                 f"针对「{prompt[:60]}...」：\n"
                 f"- 视觉检测确认：产品表面存在轻微划痕（等级：minor）\n"
                 f"- 仪表读数：0.65 MPa（正常范围）\n"
                 f"- 安全状态：区域内人员安全装备齐全\n\n"
                 f"综合评估：巡检结果合格，轻微问题需跟踪观察。",
            model='ernie-4.0-8k (mock)',
            provider='ernie',
            tokens_used=0,
        )
