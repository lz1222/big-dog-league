"""非机械臂任务 ``/mission/start`` 的双 ACK 交付契约。

该模块把一次逻辑任务请求与底层 DDS 的有限重传分离：路线和循线器都公开
已离开 WAIT_START 后才返回成功。它只发布布尔 start 消息，不接触运动、SDK
或任何实体执行器。
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any, Dict, Optional


class MissionStartDeliveryState:
    """保存一次 start 交付的 ACK；首个有效 run_id 在本请求内不可改写。"""

    def __init__(self, max_transport_publishes: int) -> None:
        if max_transport_publishes < 1:
            raise ValueError('max_transport_publishes must be at least one')
        self.logical_request_count = 1
        self.max_transport_publishes = max_transport_publishes
        self.transport_publish_count = 0
        self.discovered_subscriber_count = 0
        self.route_start_ack = False
        self.follower_start_ack = False
        self.route_run_id = ''
        self.start_delivery_completed = False
        self.start_delivery_failed = False
        self.start_delivery_error = ''

    def _fail(self, reason: str) -> None:
        """首个安全失败原因锁存，防止后续观测伪装成交付成功。"""
        if not self.start_delivery_failed:
            self.start_delivery_failed = True
            self.start_delivery_error = reason

    def record_transport_publish(self, subscriber_count: int) -> bool:
        """记录一条实际 DDS 传输；超过上限必须 fail-closed。"""
        self.discovered_subscriber_count = max(0, int(subscriber_count))
        if self.start_delivery_completed or self.start_delivery_failed:
            return False
        if self.transport_publish_count >= self.max_transport_publishes:
            self._fail('transport_publish_limit_reached')
            return False
        self.transport_publish_count += 1
        return True

    def observe_route(self, payload: Dict[str, Any]) -> None:
        """路线 ACK 必须同时证明 mission 已启动、run_id 已建立且不在等待态。"""
        if self.start_delivery_failed:
            return
        run_id = str(payload.get('run_id', '')).strip()
        phase = str(payload.get('route_phase', '')).strip()
        state = str(payload.get('state', '')).strip()
        accepted = (
            payload.get('mission_started') is True
            and bool(run_id)
            and phase not in ('', 'WAIT_START', 'EMERGENCY_STOP', 'FAULTED')
            and state not in ('FAULTED', 'EMERGENCY_STOP')
        )
        if not accepted:
            return
        if self.route_run_id and self.route_run_id != run_id:
            self._fail('route_run_id_changed')
            return
        self.route_run_id = run_id
        self.route_start_ack = True
        self._update_completion()

    def observe_follower(self, payload: Dict[str, Any]) -> None:
        """循线 ACK 要求其接受同次 mission start 并离开 WAIT_START。"""
        if self.start_delivery_failed:
            return
        nav_state = str(payload.get('nav_state', '')).strip()
        accepted = (
            payload.get('mission_started') is True
            and nav_state not in ('', 'WAIT_START', 'FAULTED', 'EMERGENCY_STOP')
        )
        if accepted:
            self.follower_start_ack = True
            self._update_completion()

    def _update_completion(self) -> None:
        if self.route_start_ack and self.follower_start_ack:
            self.start_delivery_completed = True

    def fail_timeout(self) -> None:
        """在 deadline 到期时锁存失败；ACK 不完整时绝不放行。"""
        if not self.start_delivery_completed:
            self._fail('start_delivery_ack_timeout')

    def as_dict(self) -> Dict[str, Any]:
        """输出只读诊断字段，供 smoke 与正式调用端记录同一交付事实。"""
        return {
            'logical_request_count': self.logical_request_count,
            'transport_publish_count': self.transport_publish_count,
            'discovered_subscriber_count': self.discovered_subscriber_count,
            'route_start_ack': self.route_start_ack,
            'follower_start_ack': self.follower_start_ack,
            'route_run_id': self.route_run_id,
            'start_delivery_completed': self.start_delivery_completed,
            'start_delivery_failed': self.start_delivery_failed,
            'start_delivery_error': self.start_delivery_error,
        }


def _json_object(text: str) -> Optional[Dict[str, Any]]:
    try:
        value = json.loads(text)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None


def deliver_start(
    timeout_sec: float,
    min_subscribers: int,
    max_transport_publishes: int,
    retransmit_interval_sec: float,
) -> MissionStartDeliveryState:
    """执行一次有界交付；DDS 重传只补偿发现延迟，ACK 才决定成功。"""
    if timeout_sec <= 0.0 or retransmit_interval_sec <= 0.0:
        raise ValueError('delivery timeouts must be positive')
    if min_subscribers < 1:
        raise ValueError('min_subscribers must be at least one')

    import rclpy
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, String

    # 明确禁止瞬态持久化：start 只对当时已发现的两个消费者交付。
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    delivery = MissionStartDeliveryState(max_transport_publishes)
    rclpy.init()
    node = rclpy.create_node('competition_start_delivery_{}'.format(os.getpid()))
    try:
        publisher = node.create_publisher(Bool, '/mission/start', qos)

        def on_route(message: String) -> None:
            payload = _json_object(message.data)
            if payload is not None:
                delivery.observe_route(payload)

        def on_follower(message: String) -> None:
            payload = _json_object(message.data)
            if payload is not None:
                delivery.observe_follower(payload)

        node.create_subscription(String, '/mission/line_course_state', on_route, qos)
        node.create_subscription(
            String, '/navigation/line_follow_status', on_follower, qos
        )
        deadline = time.monotonic() + timeout_sec
        next_publish_at = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            subscribers = publisher.get_subscription_count()
            delivery.discovered_subscriber_count = max(0, int(subscribers))
            if (
                subscribers >= min_subscribers
                and now >= next_publish_at
                and not delivery.start_delivery_completed
            ):
                if not delivery.record_transport_publish(subscribers):
                    break
                publisher.publish(Bool(data=True))
                next_publish_at = now + retransmit_interval_sec
            if delivery.start_delivery_completed:
                break
            if delivery.transport_publish_count >= max_transport_publishes \
                    and now >= next_publish_at:
                delivery._fail('transport_publish_limit_reached')
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if not delivery.start_delivery_completed and not delivery.start_delivery_failed:
            delivery.fail_timeout()
        return delivery
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[list] = None) -> int:
    """命令行入口：打印单行 JSON，供 shell 与验收日志无歧义解析。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout-sec', type=float, required=True)
    parser.add_argument('--min-subscribers', type=int, required=True)
    parser.add_argument('--max-transport-publishes', type=int, default=3)
    parser.add_argument('--retransmit-interval-sec', type=float, default=0.35)
    args = parser.parse_args(argv)
    delivery = deliver_start(
        args.timeout_sec,
        args.min_subscribers,
        args.max_transport_publishes,
        args.retransmit_interval_sec,
    )
    print(json.dumps(delivery.as_dict(), ensure_ascii=True, separators=(',', ':')))
    return 0 if delivery.start_delivery_completed else 1


if __name__ == '__main__':
    raise SystemExit(main())
