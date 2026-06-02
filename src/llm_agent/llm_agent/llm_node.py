"""
LLM Agent ROS2 Node — intelligent Q&A service for industrial inspection.

Provides:
  /llm/query          — text + optional images → LLM response
  /llm/generate_report — patrol results → Markdown report
  /llm/kb_search      — query industrial knowledge base

Supports multiple LLM backends with graceful fallback to mock mode.
"""
import asyncio
import logging
import os
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from inspection_msgs.srv import LLMQuery

from .providers.base_provider import BaseLLMProvider, LLMResponse, ChatMessage
from .providers.qwen_provider import QwenProvider
from .providers.ernie_provider import ErnieProvider
from .providers.deepseek_provider import DeepSeekProvider
from .industrial_kb import IndustrialKnowledgeBase
from .multimodal_analyzer import MultimodalAnalyzer
from .report_generator import ReportGenerator

logger = logging.getLogger(__name__)


class LLMAgentNode(Node):
    """Main LLM agent ROS2 node."""

    def __init__(self):
        super().__init__('llm_agent_node')

        # Parameters
        self.declare_parameter('default_provider', 'qwen')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('report_output_dir', '/tmp/inspection_reports')

        default_provider = self.get_parameter('default_provider').value
        report_dir = self.get_parameter('report_output_dir').value

        # Bridge for image conversion
        self._bridge = CvBridge()

        # Initialize providers
        self._providers: dict[str, BaseLLMProvider] = {}
        self._init_providers()

        # Set active provider
        self._active = default_provider
        if self._active not in self._providers:
            available = list(self._providers.keys())
            self._active = available[0] if available else 'qwen'

        # Knowledge base
        kb_file = None
        kb_search_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'knowledge_base.json'),
            os.path.expanduser('~/.inspection_robot/knowledge_base.json'),
        ]
        for p in kb_search_paths:
            if os.path.exists(p):
                kb_file = p
                break

        self._kb = IndustrialKnowledgeBase(knowledge_file=kb_file)

        # Multi-modal analyzer (uses active provider)
        self._analyzer = MultimodalAnalyzer(self.provider)

        # Report generator
        self._report_gen = ReportGenerator(
            provider=self.provider, output_dir=report_dir)

        # Services
        self._query_srv = self.create_service(
            LLMQuery, '/llm/query',
            self._query_callback, callback_group=ReentrantCallbackGroup())

        # Threading for async LLM calls
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._start_event_loop()

        self.get_logger().info(
            f'LLM Agent ready (provider={self._active}, '
            f'model={self.provider.model}, '
            f'configured={self.provider.is_configured()}, '
            f'kb_docs={self._kb.get_stats()["documents"]})')

    def _init_providers(self):
        """Initialize all configured LLM providers from ROS params."""
        # Declare per-provider params
        for name in ['qwen', 'ernie', 'deepseek']:
            self.declare_parameter(f'{name}.api_key', '')
            self.declare_parameter(f'{name}.api_base', '')
            self.declare_parameter(f'{name}.model', '')
            self.declare_parameter(f'{name}.max_tokens', 4096)
            self.declare_parameter(f'{name}.secret_key', '')  # For Ernie

        # Init Qwen
        qwen_key = self._get_param('qwen.api_key', 'QWEN_API_KEY')
        qwen_base = self.get_parameter('qwen.api_base').value
        qwen_model = self.get_parameter('qwen.model').value
        self._providers['qwen'] = QwenProvider(
            api_key=qwen_key, api_base=qwen_base, model=qwen_model)

        # Init Ernie
        ernie_key = self._get_param('ernie.api_key', 'ERNIE_API_KEY')
        ernie_secret = self._get_param('ernie.secret_key', 'ERNIE_SECRET_KEY')
        ernie_model = self.get_parameter('ernie.model').value
        self._providers['ernie'] = ErnieProvider(
            api_key=ernie_key, secret_key=ernie_secret, model=ernie_model)

        # Init DeepSeek
        ds_key = self._get_param('deepseek.api_key', 'DEEPSEEK_API_KEY')
        ds_base = self.get_parameter('deepseek.api_base').value
        ds_model = self.get_parameter('deepseek.model').value
        self._providers['deepseek'] = DeepSeekProvider(
            api_key=ds_key, api_base=ds_base, model=ds_model)

        configured = [k for k, v in self._providers.items() if v.is_configured()]
        self.get_logger().info(
            f'Providers: {len(configured)}/{len(self._providers)} configured '
            f'({", ".join(configured) if configured else "using mock mode"})')

    @property
    def provider(self) -> BaseLLMProvider:
        return self._providers.get(self._active, self._providers['qwen'])

    # ── ROS2 Service Callbacks ─────────────────────────────────

    def _query_callback(self, request, response):
        """Handle /llm/query service call (synchronous).

        Delegates to async LLM backend via the event loop.
        """
        query = request.query
        context = request.context
        images = request.images

        # Extract image bytes from ROS Image messages
        image_bytes_list = []
        for img_msg in (images or []):
            try:
                cv_img = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                import cv2
                _, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                image_bytes_list.append(buf.tobytes())
            except Exception as e:
                self.get_logger().error(f'Image conversion error: {e}')

        # Build knowledge-enhanced prompt
        full_query = query
        kb_context = self._kb.build_context(query, max_chunks=3)
        if kb_context:
            full_query = (
                f'{query}\n\n'
                f'参考资料（工业知识库）：\n{kb_context}'
            )
        if context:
            full_query = f'{full_query}\n\n巡检上下文：{context}'

        # Run async query synchronously
        async def _do():
            if image_bytes_list:
                return await self.provider.chat_with_image(
                    prompt=full_query, images=image_bytes_list)
            return await self.provider.chat(
                messages=[ChatMessage(role='user', content=full_query)])

        result = self._run_async(_do())
        if result:
            response.response = result.text
            response.success = result.ok
            response.error_message = result.error or ''
        else:
            response.response = '[LLM service unavailable]'
            response.success = False
            response.error_message = 'Request timed out or event loop not started'
        return response

    # ── Event Loop for Async ───────────────────────────────────

    def _start_event_loop(self):
        """Start a dedicated asyncio event loop in a background thread."""
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._run_event_loop, daemon=True)
        self._loop_thread.start()

    def _run_event_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _run_async(self, coro, timeout: float = 30.0):
        """Run an async coroutine in the dedicated event loop and wait for result."""
        if self._loop is None:
            return None
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        try:
            return future.result(timeout=timeout)
        except asyncio.TimeoutError:
            self.get_logger().error(f'LLM request timed out after {timeout}s')
            return None
        except Exception as e:
            self.get_logger().error(f'LLM request error: {e}')
            return None

    # ── Public API ─────────────────────────────────────────────

    def query(self, text: str, images: list[bytes] | None = None,
              context: str = '', timeout: float = 30.0) -> Optional[LLMResponse]:
        """Synchronous wrapper for LLM query (for use by other nodes).

        Args:
            text: User query text.
            images: Optional list of image bytes.
            context: Optional inspection context.
            timeout: Max wait in seconds.

        Returns:
            LLMResponse or None on failure.
        """
        async def _do_query():
            provider = self.provider
            if images:
                return await provider.chat_with_image(prompt=text, images=images)
            return await provider.chat(
                messages=[ChatMessage(role='user', content=text)])

        return self._run_async(_do_query(), timeout)

    def generate_report(self, patrol_summary: dict,
                        inspection_results: list[dict],
                        alerts: list[dict] | None = None) -> Optional[str]:
        """Generate an inspection report.

        Args:
            patrol_summary: Patrol mission summary.
            inspection_results: Per-waypoint detection results.
            alerts: Optional alert list.

        Returns:
            Markdown report string or None.
        """
        async def _do_report():
            return await self._report_gen.generate_report(
                patrol_summary, inspection_results, alerts or [])

        return self._run_async(_do_report(), timeout=60.0)

    def search_knowledge(self, query: str, top_k: int = 5) -> list[dict]:
        """Search the industrial knowledge base."""
        return self._kb.search(query, top_k=top_k)

    def get_provider_info(self) -> dict:
        """Return info about all providers."""
        return {
            name: p.get_model_info()
            for name, p in self._providers.items()
        }

    # ── Helpers ────────────────────────────────────────────────

    def _get_param(self, param_name: str, env_var: str = '') -> str:
        """Get parameter value, with environment variable fallback."""
        value = self.get_parameter(param_name).value
        if not value and env_var:
            value = os.environ.get(env_var, '')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = LLMAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
