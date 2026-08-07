"""非机械臂任务 ``/mission/start`` 的双 ACK 交付契约。

一次逻辑任务请求可进行有限 DDS 重传，但只有路线和循线器均已确认才成功。
发送前会确认两个必须订阅者的图端点身份，并预热两个状态流；因此测试输入
发布器或其他观察者不能凭订阅数量替代真正的控制消费者。本模块不接触运动、
SDK 或任何实体执行器。
"""

from __future__ import annotations

import argparse
import json
import os
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple


ROUTE_NODE_NAME = 'line_course_mission_node'
FOLLOWER_NODE_NAME = 'line_follower_node'


def _normal_node_name(value: Any) -> str:
    return str(value).strip().strip('/')


def _normal_namespace(value: Any) -> str:
    namespace = str(value).strip()
    if not namespace or namespace == '/':
        return ''
    return '/' + namespace.strip('/')


def _gid_text(value: Any) -> Optional[str]:
    """将 Foxy endpoint GID 变成稳定文本；不可读取时返回 None。"""
    try:
        raw = bytes(value)
    except (TypeError, ValueError):
        return None
    return raw.hex() if raw else None


def _qos_is_reliable_volatile(endpoint: Any) -> bool:
    """只接受与 start publisher 相容的 RELIABLE/VOLATILE endpoint QoS。"""
    try:
        profile = endpoint.qos_profile
        reliability = str(profile.reliability).upper()
        durability = str(profile.durability).upper()
    except (AttributeError, TypeError, ValueError):
        return False
    return 'RELIABLE' in reliability and 'VOLATILE' in durability


def endpoint_record(endpoint: Any, require_start_qos: bool) -> Optional[Dict[str, str]]:
    """提取图端点身份；读取失败绝不降级为匿名订阅者。"""
    try:
        name = _normal_node_name(endpoint.node_name)
        namespace = _normal_namespace(endpoint.node_namespace)
        gid = _gid_text(endpoint.endpoint_gid)
    except (AttributeError, TypeError, ValueError):
        return None
    if not name or gid is None:
        return None
    if require_start_qos and not _qos_is_reliable_volatile(endpoint):
        return None
    return {'node_name': name, 'node_namespace': namespace, 'gid': gid}


def unique_endpoint_records(
    endpoints: Iterable[Any], require_start_qos: bool,
) -> Optional[List[Dict[str, str]]]:
    """按 GID 去重图端点；任一身份不可读均 fail-closed。"""
    records = []
    seen_gids = set()
    for endpoint in endpoints:
        record = endpoint_record(endpoint, require_start_qos)
        if record is None:
            return None
        if record['gid'] in seen_gids:
            continue
        seen_gids.add(record['gid'])
        records.append(record)
    return records


def records_contain_node(records: Iterable[Dict[str, str]], node_name: str) -> bool:
    """严格按 node_name 和 namespace 识别正式消费者，不以总数猜测。"""
    expected = _normal_node_name(node_name)
    return any(
        _normal_node_name(record.get('node_name')) == expected
        and _normal_namespace(record.get('node_namespace')) == ''
        for record in records
    )


class MissionStartDeliveryState:
    """保存一次 start 交付的图发现、预热、ACK 与首个 run_id。"""

    def __init__(self, max_transport_publishes: int) -> None:
        if max_transport_publishes < 1:
            raise ValueError('max_transport_publishes must be at least one')
        self.logical_request_count = 1
        self.max_transport_publishes = max_transport_publishes
        self.transport_publish_count = 0
        self.discovered_subscriber_count = 0
        self.required_route_subscriber_discovered = False
        self.required_follower_subscriber_discovered = False
        self.discovered_start_subscriber_nodes = []
        self.discovered_start_subscriber_gids = []
        self.start_endpoint_discovery_completed = False
        self.route_status_source_discovered = False
        self.follower_status_source_discovered = False
        self.route_status_stream_observed = False
        self.follower_status_stream_observed = False
        self.route_start_ack = False
        self.follower_start_ack = False
        self.route_run_id = ''
        self.transport_publish_limit_reached = False
        self.missing_ack_nodes = [ROUTE_NODE_NAME, FOLLOWER_NODE_NAME]
        self.ack_wait_after_last_publish_sec = 0.0
        self._last_publish_at = None
        self.start_delivery_completed = False
        self.start_delivery_failed = False
        self.start_delivery_error = ''

    def _fail(self, reason: str) -> None:
        """首个安全失败原因锁存，防止后续观测伪装成交付成功。"""
        if not self.start_delivery_failed:
            self.start_delivery_failed = True
            self.start_delivery_error = reason

    def observe_start_subscribers(
        self, records: Optional[List[Dict[str, str]]], subscriber_count: int,
    ) -> None:
        """更新 start 订阅图；已发现必需端点随后丢失必须立即拒绝。"""
        self.discovered_subscriber_count = max(0, int(subscriber_count))
        if self.start_delivery_failed or self.start_delivery_completed:
            return
        if records is None:
            self._fail('required_start_subscriber_identity_unreadable')
            return
        self.discovered_start_subscriber_nodes = sorted({
            '{}{}'.format(
                record['node_namespace'],
                '/' + record['node_name'],
            ) for record in records
        })
        self.discovered_start_subscriber_gids = sorted({
            record['gid'] for record in records
        })
        route_present = records_contain_node(records, ROUTE_NODE_NAME)
        follower_present = records_contain_node(records, FOLLOWER_NODE_NAME)
        if self.start_endpoint_discovery_completed and (
            not route_present or not follower_present
        ):
            self._fail('required_start_subscriber_lost')
            return
        self.required_route_subscriber_discovered = route_present
        self.required_follower_subscriber_discovered = follower_present
        self.start_endpoint_discovery_completed = route_present and follower_present

    def observe_status_sources(
        self,
        route_records: Optional[List[Dict[str, str]]],
        follower_records: Optional[List[Dict[str, str]]],
    ) -> None:
        """确认两个状态 topic 由对应真实节点发布，不能用伪造状态预热。"""
        if self.start_delivery_failed or self.start_delivery_completed:
            return
        if route_records is None or follower_records is None:
            self._fail('required_status_publisher_identity_unreadable')
            return
        self.route_status_source_discovered = records_contain_node(
            route_records, ROUTE_NODE_NAME,
        )
        self.follower_status_source_discovered = records_contain_node(
            follower_records, FOLLOWER_NODE_NAME,
        )

    def ready_to_publish(self) -> bool:
        """首次及重传前的四重门控，避免只凭订阅数量抢跑。"""
        return (
            not self.start_delivery_failed
            and not self.start_delivery_completed
            and self.required_route_subscriber_discovered
            and self.required_follower_subscriber_discovered
            and self.route_status_stream_observed
            and self.follower_status_stream_observed
        )

    def record_transport_publish(
        self, subscriber_count: int, now: Optional[float] = None,
    ) -> bool:
        """记录一条实际 DDS 传输；达到上限只停止发送，不提前判失败。"""
        self.discovered_subscriber_count = max(0, int(subscriber_count))
        if not self.ready_to_publish():
            return False
        if self.transport_publish_count >= self.max_transport_publishes:
            self.transport_publish_limit_reached = True
            return False
        self.transport_publish_count += 1
        self._last_publish_at = time.monotonic() if now is None else float(now)
        if self.transport_publish_count >= self.max_transport_publishes:
            self.transport_publish_limit_reached = True
        return True

    def update_ack_wait(self, now: Optional[float] = None) -> None:
        """把最后一次发送后的只读等待时间写入诊断，不影响 timeout。"""
        if self._last_publish_at is not None:
            current = time.monotonic() if now is None else float(now)
            self.ack_wait_after_last_publish_sec = max(
                0.0, current - self._last_publish_at,
            )

    def observe_route(self, payload: Dict[str, Any]) -> None:
        """路线状态流预热，并在其接受 start 后锁存唯一 run_id。"""
        if self.start_delivery_failed:
            return
        if self.route_status_source_discovered:
            self.route_status_stream_observed = True
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
        """循线状态流预热，并在其接受同次 mission start 后确认 ACK。"""
        if self.start_delivery_failed:
            return
        if self.follower_status_source_discovered:
            self.follower_status_stream_observed = True
        nav_state = str(payload.get('nav_state', '')).strip()
        accepted = (
            payload.get('mission_started') is True
            and nav_state not in ('', 'WAIT_START', 'FAULTED', 'EMERGENCY_STOP')
        )
        if accepted:
            self.follower_start_ack = True
            self._update_completion()

    def _update_completion(self) -> None:
        self.missing_ack_nodes = []
        if not self.route_start_ack:
            self.missing_ack_nodes.append(ROUTE_NODE_NAME)
        if not self.follower_start_ack:
            self.missing_ack_nodes.append(FOLLOWER_NODE_NAME)
        if not self.missing_ack_nodes:
            self.start_delivery_completed = True

    def fail_timeout(self) -> None:
        """仅整体 deadline 到期才因 ACK 不完整失败，并明确缺失节点。"""
        self.update_ack_wait()
        if not self.start_delivery_completed:
            self._update_completion()
            self._fail('start_delivery_ack_timeout')

    def as_dict(self) -> Dict[str, Any]:
        """输出只读交付诊断，供 smoke 与正式调用端记录同一事实。"""
        return {
            'logical_request_count': self.logical_request_count,
            'transport_publish_count': self.transport_publish_count,
            'discovered_subscriber_count': self.discovered_subscriber_count,
            'required_route_subscriber_discovered': self.required_route_subscriber_discovered,
            'required_follower_subscriber_discovered': self.required_follower_subscriber_discovered,
            'discovered_start_subscriber_nodes': self.discovered_start_subscriber_nodes,
            'discovered_start_subscriber_gids': self.discovered_start_subscriber_gids,
            'start_endpoint_discovery_completed': self.start_endpoint_discovery_completed,
            'route_status_stream_observed': self.route_status_stream_observed,
            'follower_status_stream_observed': self.follower_status_stream_observed,
            'route_start_ack': self.route_start_ack,
            'follower_start_ack': self.follower_start_ack,
            'route_run_id': self.route_run_id,
            'transport_publish_limit_reached': self.transport_publish_limit_reached,
            'missing_ack_nodes': self.missing_ack_nodes,
            'ack_wait_after_last_publish_sec': self.ack_wait_after_last_publish_sec,
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


def _topic_records(
    node: Any, method_name: str, topic_name: str, require_start_qos: bool,
) -> Optional[List[Dict[str, str]]]:
    """从 Foxy 图查询取端点快照；查询异常也必须 fail-closed。"""
    try:
        endpoints = getattr(node, method_name)(topic_name)
    except (AttributeError, RuntimeError, TypeError, ValueError):
        return None
    return unique_endpoint_records(endpoints, require_start_qos)


def deliver_start(
    timeout_sec: float,
    min_subscribers: int,
    max_transport_publishes: int,
    retransmit_interval_sec: float,
) -> MissionStartDeliveryState:
    """执行一次有界交付；端点和状态流齐备后才发布，ACK 决定成功。"""
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

    # 明确禁止瞬态持久化：start 仅对已发现的两个正式消费者交付。
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
            delivery.observe_start_subscribers(
                _topic_records(
                    node, 'get_subscriptions_info_by_topic', '/mission/start', True,
                ),
                subscribers,
            )
            delivery.observe_status_sources(
                _topic_records(
                    node, 'get_publishers_info_by_topic',
                    '/mission/line_course_state', False,
                ),
                _topic_records(
                    node, 'get_publishers_info_by_topic',
                    '/navigation/line_follow_status', False,
                ),
            )
            if delivery.start_delivery_failed or delivery.start_delivery_completed:
                break
            delivery.update_ack_wait(now)
            if (
                delivery.ready_to_publish()
                and delivery.transport_publish_count < max_transport_publishes
                and now >= next_publish_at
            ):
                if delivery.record_transport_publish(subscribers, now):
                    publisher.publish(Bool(data=True))
                    next_publish_at = now + retransmit_interval_sec
            # 达到发送上限后只停止 publish，仍 spin 到整体 deadline 等迟到 ACK。
            if delivery.start_delivery_completed:
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
