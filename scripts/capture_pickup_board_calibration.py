#!/usr/bin/env python3
"""只读采集 Go2 前置挡板的黄金画面统计，不修改任何生产 YAML。"""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rk_interfaces.msg import SpecialTargetDetection
from rk_mission.platform_calibration_contract import pickup_capture_result


def _summary(values):
    """输出可复核统计，空样本明确标记而非填充猜测值。"""
    if not values:
        return {'count': 0, 'calibration_required': True}
    ordered = sorted(float(value) for value in values)
    count = len(ordered)
    mean = sum(ordered) / count
    return {
        'count': count, 'mean': mean,
        'std': (sum((item - mean) ** 2 for item in ordered) / count) ** 0.5,
        'min': ordered[0], 'max': ordered[-1],
        'p05': ordered[int((count - 1) * 0.05)],
        'p50': ordered[int((count - 1) * 0.50)],
        'p95': ordered[int((count - 1) * 0.95)],
    }


class PickupBoardCapture(Node):
    """仅累计 ``pickup_board_anchor`` 正样本，绝不发布控制 topic。"""

    def __init__(self, topic):
        super().__init__('capture_pickup_board_calibration')
        self.samples = {name: [] for name in (
            'area_ratio', 'center_x_ratio', 'center_y_ratio', 'bottom_y_ratio',
            'width_ratio', 'height_ratio', 'top_y_ratio')}
        self.create_subscription(
            SpecialTargetDetection, topic, self._on_board, 10)

    def _on_board(self, message):
        if not message.visible:
            return
        self.samples['area_ratio'].append(message.area_ratio)
        self.samples['center_x_ratio'].append(message.center_x)
        self.samples['center_y_ratio'].append(message.center_y)
        self.samples['bottom_y_ratio'].append(
            message.center_y + message.height_ratio / 2.0)
        self.samples['width_ratio'].append(message.width_ratio)
        self.samples['height_ratio'].append(message.height_ratio)
        # 顶边不受本轮 bottom/image 边界饱和的直接影响，供弧顶现场复核。
        self.samples['top_y_ratio'].append(
            message.center_y - message.height_ratio / 2.0)


def main():
    """运行固定时长采集并写 JSON 证据，用户审阅后再手工写入 YAML。"""
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/perception/pickup_board_anchor')
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument(
        '--output',
        default='PICKUP_BOARD_GOLDEN_SIGNATURE.json')
    args = parser.parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit('--duration-sec must be positive')
    rclpy.init()
    node = PickupBoardCapture(args.topic)
    deadline = time.monotonic() + args.duration_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(
                    0.2, deadline - time.monotonic()))
        capture_valid, reason = pickup_capture_result(node.samples)
        payload = {
            'schema': 'pickup_board_golden_signature/v1',
            'calibration_required': True,
            # 默认 detector 使用不可检测阈值；无正样本必须明确报告而非伪造标定。
            'capture_valid': capture_valid,
            'reason': reason,
            'source_topic': args.topic,
            'duration_sec': args.duration_sec,
            'metrics': {
                key: _summary(values) for key,
                values in node.samples.items()}}
        Path(
            args.output).write_text(
            json.dumps(
                payload,
                indent=2,
                sort_keys=True) +
            '\n',
            encoding='utf-8')
        print('Wrote {}'.format(args.output))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
