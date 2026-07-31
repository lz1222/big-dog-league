#!/usr/bin/env python3

"""终端 D 中文实时提示器：只订阅 B2 JSON，不创建任何控制发布器。"""

import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from maze_operator_prompt_core import (
    build_operator_view,
    format_dashboard,
)


COLOR_BY_SEVERITY = {
    'danger': '\033[1;31m',
    'warning': '\033[1;33m',
    'caution': '\033[1;33m',
    'turn': '\033[1;36m',
    'normal': '\033[1;32m',
    'complete': '\033[1;32m',
}
COLOR_RESET = '\033[0m'
CLEAR_SCREEN = '\033[2J\033[H'


class MazeOperatorMonitor(Node):
    """把 B2 状态转换为操控员可直接执行的中文安全提示。"""

    def __init__(self):
        super().__init__('maze_operator_monitor')

        self.status_topic = self._string_parameter(
            'navigation_status_topic',
            '/maze/navigation/dry_run_status',
        )
        self.refresh_rate = self._positive_float_parameter(
            'refresh_rate',
            5.0,
        )
        self.status_stale_timeout_sec = self._positive_float_parameter(
            'status_stale_timeout_sec',
            1.50,
        )
        self.use_color = bool(
            self.declare_parameter('use_color', True).value
        ) and sys.stdout.isatty()
        self.beep_on_action_change = bool(
            self.declare_parameter(
                'beep_on_action_change',
                True,
            ).value
        ) and sys.stdout.isatty()

        self._payload = None
        self._last_receive_time = None
        self._last_action_signature = None

        # 本节点只创建 String 订阅和显示定时器，严禁添加 Twist 发布器。
        self.subscription = self.create_subscription(
            String,
            self.status_topic,
            self._on_status,
            10,
        )
        self.refresh_timer = self.create_timer(
            1.0 / self.refresh_rate,
            self._on_refresh,
        )

        self.get_logger().info(
            'Maze operator monitor ready: '
            f'topic={self.status_topic}, '
            f'stale_timeout={self.status_stale_timeout_sec:.2f}s; '
            'read-only Chinese prompts'
        )
        self._render(force=True)

    def _on_status(self, msg):
        """保存最新有效 JSON；格式错误会显示停止提示而不是退出节点。"""
        now = time.monotonic()
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError('root must be a JSON object')
        except (TypeError, ValueError) as error:
            payload = {
                'state': 'FAULT_STOP',
                'reason': f'b2_payload_invalid:{error}',
                'route_index': 0,
                'route_total': 0,
            }
        self._payload = payload
        self._last_receive_time = now
        self._render(force=False)

    def _on_refresh(self):
        """周期刷新面板，并独立检测 B2 Topic 是否已经断流。"""
        self._render(force=True)

    def _render(self, force):
        now = time.monotonic()
        stream_age = (
            math.inf
            if self._last_receive_time is None
            else max(0.0, now - self._last_receive_time)
        )
        view = build_operator_view(
            self._payload,
            stream_age,
            self.status_stale_timeout_sec,
        )
        signature = (
            view.action_code,
            view.action_title,
            str((self._payload or {}).get('state')),
            str((self._payload or {}).get('reason')),
            (self._payload or {}).get('route_index'),
        )
        changed = signature != self._last_action_signature
        if not force and not changed:
            return
        self._last_action_signature = signature

        dashboard = format_dashboard(
            self._payload,
            view,
            stream_age,
        )
        if self.use_color:
            color = COLOR_BY_SEVERITY.get(view.severity, '')
            prefix = '\a' if changed and self.beep_on_action_change else ''
            output = (
                f'{prefix}{CLEAR_SCREEN}{color}'
                f'{dashboard}{COLOR_RESET}'
            )
        else:
            # 非交互重定向时只在操作变化时输出，避免日志每秒刷屏。
            if not changed:
                return
            output = dashboard
        print(output, flush=True)

    def _string_parameter(self, name, default):
        value = str(self.declare_parameter(name, default).value)
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive_float_parameter(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be positive and finite')
        return value


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MazeOperatorMonitor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
