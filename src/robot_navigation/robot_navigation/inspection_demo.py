#!/usr/bin/env python3
"""
Automated Inspection Demo — runs a complete patrol and generates a report.

This script:
1. Waits for patrol server to be ready
2. Sends a patrol mission with predefined inspection waypoints
3. Monitors progress in real-time
4. Collects simulated inspection results
5. Generates an inspection report via LLM agent
6. Saves the report to disk

Usage:
    ros2 run robot_navigation inspection_demo
"""
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from inspection_msgs.action import PatrolMission


class InspectionDemo(Node):
    """Automated inspection demonstration node."""

    def __init__(self):
        super().__init__('inspection_demo')

        self._client = ActionClient(self, PatrolMission, '/patrol/execute')
        self._done = False
        self._start_time = 0.0
        self._results: list[dict] = []
        self._alerts: list[dict] = []

        self.get_logger().info('='*60)
        self.get_logger().info('  工业机器人巡检系统 - 自动化演示')
        self.get_logger().info('='*60)

    def run_demo(self):
        """Run the complete inspection demonstration."""
        # Step 1: Wait for patrol server
        self.get_logger().info('\n[1/5] Waiting for patrol server...')
        if not self._client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Patrol server not available! Is the simulation running?')
            self.get_logger().error('Run: ros2 launch robot_bringup simulation_bringup.launch.py')
            return False

        self.get_logger().info('Patrol server connected.')

        # Step 2: Define inspection waypoints
        mission = self._define_mission()
        self.get_logger().info(
            f'\n[2/5] Sending patrol mission: {len(mission["waypoints"])} waypoints')

        # Step 3: Send and execute
        self._start_time = time.time()
        goal = PatrolMission.Goal()
        goal.waypoints = mission['waypoints']
        goal.loop_mode = mission.get('loop_mode', False)
        goal.stay_duration = mission.get('stay_duration', 3.0)
        goal.inspection_modes = mission.get('inspection_modes', [])

        self._send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

        # Spin until done
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Step 4: Collect results
        self.get_logger().info('\n[4/5] Collecting inspection results...')
        self._collect_inspection_results(mission)

        # Step 5: Generate report
        self.get_logger().info('\n[5/5] Generating inspection report...')
        report = self._generate_report(mission)

        # Save report
        report_dir = os.path.expanduser('/tmp/inspection_reports')
        os.makedirs(report_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = os.path.join(report_dir, f'inspection_report_{timestamp}.md')
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report)

        self.get_logger().info(f'\nReport saved to: {report_file}')
        self.get_logger().info(f'Report size: {len(report)} characters')
        return True

    # ── Mission Definition ────────────────────────────────────

    def _define_mission(self) -> dict:
        """Define a realistic inspection patrol mission.

        Waypoints correspond to areas in the industrial_factory world:
        - WP1: Conveyor belt area (defect inspection)
        - WP2: Equipment/instrument area (meter reading)
        - WP3: Safety zone (safety compliance)
        """
        def make_pose(x, y, yaw=0.0):
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = 0.0
            p.orientation.z = math.sin(yaw / 2.0)
            p.orientation.w = math.cos(yaw / 2.0)
            return p

        return {
            'route_name': '标准工厂巡检路线',
            'loop_mode': False,
            'stay_duration': 3.0,  # Short for demo
            'waypoints': [
                make_pose(4.0, -2.5, 0.0),    # WP1: Conveyor belt (defect)
                make_pose(-3.5, 0.0, 0.0),    # WP2: Instrument panel (meter)
                make_pose(4.0, 2.5, 3.1416),  # WP3: Safety zone (safety)
            ],
            'inspection_modes': ['defect', 'meter', 'safety'],
            'wp_names': ['产线质检区 - 传送带', '设备区 - 仪表盘', '安全区 - 围栏'],
        }

    # ── Action Callbacks ──────────────────────────────────────

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Patrol mission rejected!')
            self._done = True
            return
        self.get_logger().info('Mission accepted — executing patrol...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        wp_num = fb.current_waypoint + 1 if fb.current_waypoint >= 0 else '?'
        self.get_logger().info(
            f'  [{fb.current_state}] WP{wp_num} | '
            f'dist={fb.distance_remaining:.2f}m')

    def _result_callback(self, future):
        result = future.result().result
        elapsed = time.time() - self._start_time
        self.get_logger().info(
            f'\n[3/5] Patrol completed in {elapsed:.0f}s\n'
            f'  Success: {result.success}\n'
            f'  Waypoints visited: {result.waypoints_visited}\n'
            f'  Summary: {result.summary}')
        self._done = True

    # ── Result Collection ─────────────────────────────────────

    def _collect_inspection_results(self, mission: dict):
        """Collect mock inspection results for the demo report.

        In production, these would be read from /inspection/result topic.
        """
        wp_names = mission.get('wp_names', [f'WP{i+1}' for i in range(len(mission['waypoints']))])
        inspection_modes = mission.get('inspection_modes', [])

        for i, name in enumerate(wp_names):
            mode = inspection_modes[i] if i < len(inspection_modes) else 'none'
            result = self._simulate_inspection(name, mode, i)
            self._results.append(result)
            if result.get('alerts'):
                self._alerts.extend(result['alerts'])

        total_defects = sum(r.get('defect_count', 0) for r in self._results)
        total_alerts = len(self._alerts)
        self.get_logger().info(
            f'  Results: {len(self._results)} waypoints, '
            f'{total_defects} defects, {total_alerts} alerts')

    def _simulate_inspection(self, wp_name: str, mode: str, idx: int) -> dict:
        """Simulate inspection results for demo purposes."""
        import random
        random.seed(idx + 42)

        result = {
            'name': wp_name,
            'x': random.uniform(-5, 5),
            'y': random.uniform(-5, 5),
            'defect_count': 0,
            'meter_count': 0,
            'safety_count': 0,
            'status': 'ok',
            'details': '检测通过',
            'alerts': [],
        }

        if mode == 'defect' or mode == 'all':
            # Simulate defect detection
            if random.random() < 0.4:  # 40% chance of defect
                defect_types = ['scratch', 'crack', 'stain']
                severity = random.choice(['minor', 'minor', 'major'])
                defect_type = random.choice(defect_types)
                result['defect_count'] = random.randint(1, 2)
                result['status'] = 'warning' if severity == 'minor' else 'error'
                result['details'] = f'检出{defect_type}缺陷（{severity}）'
                if severity == 'major':
                    result['alerts'].append({
                        'severity': 2,
                        'alert_type': 'defect',
                        'message': f'严重缺陷: {defect_type}（区域: {wp_name}）',
                        'suggested_actions': ['停线检查', '隔离批次', '通知质量主管'],
                    })

        if mode == 'meter' or mode == 'all':
            # Simulate meter reading
            reading = random.uniform(0.2, 0.9)
            is_anomaly = random.random() < 0.15  # 15% anomaly chance
            result['meter_count'] = random.randint(1, 3)
            if is_anomaly:
                result['status'] = 'warning'
                result['details'] = f'仪表读数异常: {reading:.2f} MPa（超出正常范围）'
                result['alerts'].append({
                    'severity': 1,
                    'alert_type': 'meter',
                    'message': f'仪表读数异常: {reading:.2f} MPa（点: {wp_name}）',
                    'suggested_actions': ['人工复核读数', '检查设备状态'],
                })
            else:
                result['details'] = f'仪表读数正常: {reading:.2f} MPa'

        if mode == 'safety' or mode == 'all':
            # Simulate safety check
            violation = random.random() < 0.25  # 25% violation chance
            result['safety_count'] = 1
            if violation:
                violation_types = ['helmet_missing', 'zone_intrusion']
                vtype = random.choice(violation_types)
                result['status'] = 'error'
                result['details'] = f'安全违规: {vtype}'
                result['alerts'].append({
                    'severity': 2,
                    'alert_type': 'safety',
                    'message': f'安全违规: {vtype}（点: {wp_name}）',
                    'suggested_actions': ['立即纠正违规行为', '记录安全事件', '通知安全主管'],
                })
            else:
                result['details'] = '安全合规，人员装备齐全'

        return result

    # ── Report Generation ─────────────────────────────────────

    def _generate_report(self, mission: dict) -> str:
        """Generate a Markdown inspection report."""
        timestamp = datetime.now()
        elapsed = time.time() - self._start_time if self._start_time else 0

        lines = []
        lines.append('# 工业巡检报告')
        lines.append('')
        lines.append(f'**报告编号**: INSP-DEMO-{timestamp.strftime("%Y%m%d-%H%M%S")}')
        lines.append(f'**生成时间**: {timestamp.strftime("%Y-%m-%d %H:%M:%S")}')
        lines.append(f'**巡检系统**: Industrial Inspection Robot (ROS2)')
        lines.append('')

        # Summary
        total_defects = sum(r.get('defect_count', 0) for r in self._results)
        total_meters = sum(r.get('meter_count', 0) for r in self._results)
        total_alerts = len(self._alerts)
        critical = sum(1 for a in self._alerts if a.get('severity', 0) >= 2)

        status = 'PASS' if critical == 0 else 'WARN' if critical < 2 else 'FAIL'

        lines.append('## 巡检概要')
        lines.append('')
        lines.append(f'| 项目 | 详情 |')
        lines.append(f'|------|------|')
        lines.append(f'| 巡检结果 | **{status}** |')
        lines.append(f'| 巡检点位 | {len(self._results)} |')
        lines.append(f'| 巡检时长 | {elapsed:.0f}秒 |')
        lines.append(f'| 巡检路线 | {mission.get("route_name", "默认路线")} |')
        lines.append(f'| 缺陷检出 | {total_defects} 处 |')
        lines.append(f'| 仪表读取 | {total_meters} 个 |')
        lines.append(f'| 告警总数 | {total_alerts} 条（{critical} 条严重） |')
        lines.append('')

        # Per-waypoint results
        lines.append('## 巡检点详情')
        lines.append('')
        for i, r in enumerate(self._results):
            icon = {'ok': ':white_check_mark:', 'warning': ':warning:',
                    'error': ':x:'}.get(r['status'], ':question:')
            lines.append(f'### {i+1}. {r["name"]} {icon}')
            lines.append(f'- 状态: **{r["status"].upper()}**')
            lines.append(f'- 缺陷检测: {r["defect_count"]} 处')
            lines.append(f'- 仪表读数: {r["meter_count"]} 个')
            lines.append(f'- 安全检测: {r["safety_count"]} 项')
            lines.append(f'- 详情: {r["details"]}')
            lines.append('')

        # Alerts
        if self._alerts:
            lines.append('## 告警详情')
            lines.append('')
            for i, a in enumerate(self._alerts):
                sev = {0: ':information_source:', 1: ':warning:',
                       2: ':red_circle:', 3: ':rotating_light:'}.get(a['severity'], '')
                lines.append(f'### {sev} 告警 {i+1}')
                lines.append(f'- 类型: `{a["alert_type"]}`')
                lines.append(f'- 描述: {a["message"]}')
                lines.append(f'- 建议: {"; ".join(a["suggested_actions"])}')
                lines.append('')

        # Footer
        lines.append('---')
        lines.append('')
        lines.append('*本报告由工业巡检机器人系统自动生成（演示模式）*')
        lines.append(f'*生成时间: {timestamp.strftime("%Y-%m-%d %H:%M:%S")}*')

        return '\n'.join(lines)


def main(args=None):
    rclpy.init(args=args)

    demo = InspectionDemo()
    success = demo.run_demo()

    if success:
        print('\n' + '='*60)
        print('  演示完成！报告已保存到 /tmp/inspection_reports/')
        print('='*60)
    else:
        print('\n演示失败。请确保仿真环境已启动。')
        sys.exit(1)

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
