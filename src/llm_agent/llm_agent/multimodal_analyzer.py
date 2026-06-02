"""
Multi-modal Analyzer — combines images + inspection data + LLM for analysis.

Takes inspection images, detection results, and user questions,
then invokes the LLM for contextual analysis of industrial scenarios.
"""
import logging
from datetime import datetime
from typing import Optional

from .providers.base_provider import BaseLLMProvider, LLMResponse, ChatMessage

logger = logging.getLogger(__name__)


class MultimodalAnalyzer:
    """Analyze inspection data using multi-modal LLM capabilities."""

    SYSTEM_PROMPT_INSPECTION = (
        '你是一个工业巡检专家。你需要分析巡检图像和检测结果，'
        '提供专业的技术评估和建议。请基于工业标准和最佳实践给出分析。'
    )

    def __init__(self, provider: BaseLLMProvider):
        self._provider = provider

    async def analyze_defect(self, image_bytes: bytes,
                             detection_result: dict,
                             question: str = '') -> LLMResponse:
        """Analyze a product defect detection result.

        Args:
            image_bytes: Image showing the defect.
            detection_result: Output from DefectDetector.
            question: Specific question (optional).

        Returns:
            LLMResponse with expert analysis.
        """
        q = question or '请分析这个产品缺陷的严重程度和可能原因'
        prompt = self._build_defect_prompt(q, detection_result)
        return await self._provider.chat_with_image(
            prompt=prompt,
            images=[image_bytes],
            system_prompt=self.SYSTEM_PROMPT_INSPECTION,
        )

    async def analyze_meter(self, image_bytes: bytes,
                            reading: dict,
                            question: str = '') -> LLMResponse:
        """Analyze an instrument meter reading.

        Args:
            image_bytes: Image of the meter.
            reading: Output from MeterReader.
            question: Specific question (optional).

        Returns:
            LLMResponse with reading interpretation.
        """
        q = question or '请判断这个仪表读数是否正常并给出建议'
        prompt = self._build_meter_prompt(q, reading)
        return await self._provider.chat_with_image(
            prompt=prompt,
            images=[image_bytes],
            system_prompt=self.SYSTEM_PROMPT_INSPECTION,
        )

    async def analyze_safety(self, image_bytes: bytes,
                             violation: dict,
                             question: str = '') -> LLMResponse:
        """Analyze a safety violation.

        Args:
            image_bytes: Image of the safety scene.
            violation: Output from SafetyChecker.
            question: Specific question (optional).

        Returns:
            LLMResponse with safety assessment.
        """
        q = question or '请评估这个安全违规情况的严重性并建议整改措施'
        prompt = self._build_safety_prompt(q, violation)
        return await self._provider.chat_with_image(
            prompt=prompt,
            images=[image_bytes],
            system_prompt=self.SYSTEM_PROMPT_INSPECTION,
        )

    async def analyze_scene(self, images: list[bytes],
                            all_results: dict,
                            question: str = '') -> LLMResponse:
        """Comprehensive multi-image scene analysis.

        Args:
            images: Multiple inspection images from a patrol run.
            all_results: Aggregated results from all detectors.
            question: User's question.

        Returns:
            LLMResponse with comprehensive analysis.
        """
        q = question or '请综合分析本次巡检采集的图像，给出整体评估'
        prompt = (
            f'{q}\n\n'
            f'本次巡检共计{len(images)}张图像，检测结果汇总：\n'
            f'- 缺陷检测：{all_results.get("defect_count", 0)}处\n'
            f'- 仪表读数：{all_results.get("meter_count", 0)}个\n'
            f'- 安全检测：发现{all_results.get("safety_violations", 0)}项违规\n\n'
            f'请基于以上数据给出综合分析和建议。'
        )

        if len(images) == 1:
            return await self._provider.chat_with_image(
                prompt=prompt, images=images,
                system_prompt=self.SYSTEM_PROMPT_INSPECTION)

        # Multi-image: use first image with detailed prompt
        return await self._provider.chat_with_image(
            prompt=prompt,
            images=images[:3],  # Limit to 3 images to avoid token overflow
            system_prompt=self.SYSTEM_PROMPT_INSPECTION,
        )

    # ── Prompt Builders ────────────────────────────────────────

    @staticmethod
    def _build_defect_prompt(question: str, result: dict) -> str:
        parts = [
            question,
            '',
            '【检测数据】',
            f'- 缺陷类型: {result.get("defect_type", result.get("label", "未知"))}',
            f'- 严重程度: {result.get("severity", "未评估")}',
            f'- 置信度: {result.get("confidence", 0):.1%}',
            f'- 缺陷面积: {result.get("defect_area", result.get("area", "未知"))}',
            f'- 检测时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            '',
            '请回答：',
            '1. 缺陷的严重程度评估',
            '2. 可能的原因分析',
            '3. 建议的处理措施',
            '4. 是否需要立即停线处理',
        ]
        return '\n'.join(parts)

    @staticmethod
    def _build_meter_prompt(question: str, reading: dict) -> str:
        parts = [
            question,
            '',
            '【仪表读数】',
            f'- 仪表类型: {reading.get("meter_type", "未知")}',
            f'- 当前读数: {reading.get("reading_value", "N/A")} '
            f'{reading.get("reading_unit", "")}',
            f'- 正常范围: {reading.get("min_normal", 0)} - '
            f'{reading.get("max_normal", 100)} {reading.get("reading_unit", "")}',
            f'- 是否异常: {"是⚠️" if reading.get("is_anomaly") else "否✅"}',
            f'- 置信度: {reading.get("confidence", 0):.1%}',
            '',
            '请回答：',
            '1. 读数是否在安全范围内',
            '2. 如异常，可能的原因和影响',
            '3. 建议的操作或维护措施',
        ]
        return '\n'.join(parts)

    @staticmethod
    def _build_safety_prompt(question: str, violation: dict) -> str:
        parts = [
            question,
            '',
            '【安全违规信息】',
            f'- 违规类型: {violation.get("check_type", violation.get("label", "未知"))}',
            f'- 严重程度: {violation.get("severity", "未评估")}',
            f'- 涉及人数: {violation.get("person_count", "未知")}',
            f'- 置信度: {violation.get("confidence", 0):.1%}',
            '',
            '请回答：',
            '1. 安全风险评估',
            '2. 是否属于重大安全隐患',
            '3. 整改建议和紧急措施',
            '4. 是否需要立即疏散或停工',
        ]
        return '\n'.join(parts)
