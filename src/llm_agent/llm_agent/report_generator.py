"""
Inspection Report Generator — creates structured Markdown reports.

Collects all inspection results from a patrol mission and generates
professional reports with summary, per-waypoint details, anomaly
highlighting, and AI-generated analysis.
"""
import logging
import os
from datetime import datetime
from typing import Optional

from .providers.base_provider import BaseLLMProvider, LLMResponse, ChatMessage

logger = logging.getLogger(__name__)


class ReportGenerator:
    """Generate structured inspection reports from patrol data."""

    def __init__(self, provider: Optional[BaseLLMProvider] = None,
                 output_dir: str = '/tmp/inspection_reports'):
        self._provider = provider
        self._output_dir = os.path.expanduser(output_dir)
        os.makedirs(self._output_dir, exist_ok=True)

    async def generate_report(self,
                              patrol_summary: dict,
                              inspection_results: list[dict],
                              alerts: list[dict] | None = None,
                              use_ai: bool = True) -> str:
        """Generate a complete inspection report in Markdown.

        Args:
            patrol_summary: Patrol mission summary
                {waypoints_visited, total_waypoints, duration, route_name}.
            inspection_results: List of per-waypoint detection results.
            alerts: List of alerts raised during patrol.
            use_ai: Whether to include AI-generated analysis section.

        Returns:
            Full Markdown report as a string.
        """
        timestamp = datetime.now()
        report_id = timestamp.strftime('INSP-%Y%m%d-%H%M%S')

        sections = []

        # Header
        sections.append(self._build_header(report_id, timestamp))

        # Executive Summary
        sections.append(self._build_summary(patrol_summary, inspection_results, alerts or []))

        # Patrol Route Info
        sections.append(self._build_route_info(patrol_summary))

        # Per-Waypoint Results
        sections.append(self._build_waypoint_results(inspection_results))

        # Alerts & Anomalies
        if alerts:
            sections.append(self._build_alert_section(alerts))

        # AI Analysis (optional)
        if use_ai and self._provider and self._provider.is_configured():
            ai_section = await self._generate_ai_analysis(
                patrol_summary, inspection_results, alerts or [])
            sections.append(ai_section)
        elif use_ai:
            sections.append(self._build_ai_placeholder())

        # Footer
        sections.append(self._build_footer())

        report = '\n\n---\n\n'.join(sections)

        # Save to file
        self._save_report(report_id, report)

        return report

    def generate_template_report(self,
                                 patrol_summary: dict,
                                 inspection_results: list[dict],
                                 alerts: list[dict] | None = None) -> str:
        """Generate report without LLM (template-based, synchronous)."""
        import asyncio
        loop = asyncio.new_event_loop()
        try:
            return loop.run_until_complete(
                self.generate_report(patrol_summary, inspection_results, alerts, use_ai=False))
        finally:
            loop.close()

    @staticmethod
    def _fmt(value) -> str:
        """Format a value for display, handling both numbers and strings."""
        if isinstance(value, float):
            return f'{value:.2f}'
        return str(value)

    # ── Section Builders ───────────────────────────────────────

    def _build_header(self, report_id: str, timestamp: datetime) -> str:
        return (
            f'# 工业巡检报告\n\n'
            f'**报告编号**: {report_id}  \n'
            f'**生成时间**: {timestamp.strftime("%Y-%m-%d %H:%M:%S")}  \n'
            f'**巡检系统**: Industrial Inspection Robot (ROS2)  \n'
            f'**报告状态**: {"草稿" if timestamp.second % 2 == 0 else "正式"}\n'
        )

    def _build_summary(self, patrol: dict, results: list[dict],
                       alerts: list[dict]) -> str:
        total_defects = sum(
            r.get('defect_count', 0) for r in results)
        total_meters = sum(
            r.get('meter_count', 0) for r in results)
        total_safety = sum(
            r.get('safety_count', 0) for r in results)
        critical_alerts = sum(
            1 for a in alerts if a.get('severity', 0) >= 2)

        status = '✅ 合格' if critical_alerts == 0 else '⚠️ 存在异常'
        if critical_alerts >= 3:
            status = '🔴 严重异常'

        return (
            f'## 巡检概要\n\n'
            f'| 项目 | 详情 |\n'
            f'|------|------|\n'
            f'| 巡检结果 | {status} |\n'
            f'| 巡检点位 | {patrol.get("waypoints_visited", 0)}/{patrol.get("total_waypoints", 0)} |\n'
            f'| 巡检时长 | {patrol.get("duration", "未知")} |\n'
            f'| 缺陷检出 | {total_defects} 处 |\n'
            f'| 仪表读取 | {total_meters} 个 |\n'
            f'| 安全检测 | {"发现 " + str(total_safety) + " 项违规" if total_safety > 0 else "合规"} |\n'
            f'| 严重告警 | {critical_alerts} 条 |\n'
        )

    def _build_route_info(self, patrol: dict) -> str:
        route = patrol.get('route_name', '默认巡检路线')
        return (
            f'## 巡检路线\n\n'
            f'**路线名称**: {route}  \n'
            f'**巡检模式**: {"循环巡检" if patrol.get("loop_mode") else "单次巡检"}  \n'
            f'**每点停留**: {patrol.get("stay_duration", 5)}秒  \n'
        )

    def _build_waypoint_results(self, results: list[dict]) -> str:
        if not results:
            return '## 巡检点详情\n\n无巡检数据。'

        lines = ['## 巡检点详情']
        for i, wp in enumerate(results):
            wp_name = wp.get('name', f'巡检点 {i + 1}')
            status = wp.get('status', 'ok')
            status_icon = {'ok': '✅', 'warning': '⚠️', 'error': '🔴'}.get(status, '❓')

            lines.append(
                f'### {i + 1}. {wp_name} {status_icon}\n'
                f'- 位置: ({self._fmt(wp.get("x", 0))}, {self._fmt(wp.get("y", 0))})\n'
                f'- 缺陷检测: {wp.get("defect_count", 0)}处\n'
                f'- 仪表读数: {wp.get("meter_count", 0)}个\n'
                f'- 安全检测: {wp.get("safety_count", 0)}项\n'
                f'- 检测项详情: {wp.get("details", "无异常")}\n'
            )

        return '\n'.join(lines)

    def _build_alert_section(self, alerts: list[dict]) -> str:
        severity_labels = {0: 'ℹ️ 信息', 1: '⚠️ 警告', 2: '🔴 严重', 3: '🚨 紧急'}
        lines = ['## 告警与异常']

        for i, alert in enumerate(alerts):
            sev = alert.get('severity', 0)
            label = severity_labels.get(sev, f'级别{sev}')
            lines.append(
                f'### 告警 {i + 1}: {label}\n'
                f'- 类型: {alert.get("alert_type", alert.get("type", "未知"))}\n'
                f'- 描述: {alert.get("message", "无描述")}\n'
                f'- 建议措施: {", ".join(alert.get("suggested_actions", ["无"]))}\n'
            )

        return '\n'.join(lines)

    async def _generate_ai_analysis(self, patrol: dict,
                                     results: list[dict],
                                     alerts: list[dict]) -> str:
        """Use LLM to generate an AI-powered analysis section."""
        prompt = (
            '请根据以下巡检数据生成一份专业分析章节，包含：\n'
            '1. 整体巡检质量评估\n'
            '2. 主要发现的总结和优先级排序\n'
            '3. 趋势分析（如有历史数据对比）\n'
            '4. 改进建议\n'
            f'\n巡检数据：{patrol}\n检测结果：{results}\n告警：{alerts}'
        )

        response = await self._provider.chat(
            messages=[ChatMessage(role='user', content=prompt)],
            system_prompt='你是工业巡检报告撰写专家。请用中文输出专业分析。',
        )

        return (
            f'## AI 分析建议\n\n'
            f'*由 {self._provider.provider_name}/{self._provider.model} 生成*\n\n'
            f'{response.text if response.ok else "_AI分析暂时不可用_"}\n'
        )

    def _build_ai_placeholder(self) -> str:
        return (
            '## AI 分析建议\n\n'
            '*AI分析未启用 — 请配置LLM API密钥以启用智能分析*\n\n'
            '> 配置方式：编辑 `llm_config.yaml` 并设置 `QWEN_API_KEY` 环境变量\n'
        )

    def _build_footer(self) -> str:
        return (
            f'---\n\n'
            f'*本报告由工业巡检机器人系统自动生成*\n'
            f'*生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}*\n'
            f'*仅供内部使用，如有疑问请联系质量管理部门*\n'
        )

    def _save_report(self, report_id: str, report: str):
        """Save report to disk."""
        filename = os.path.join(self._output_dir, f'{report_id}.md')
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write(report)
            logger.info(f'Report saved: {filename}')
        except Exception as e:
            logger.error(f'Failed to save report: {e}')
