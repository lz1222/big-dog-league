#!/usr/bin/env python3
"""只读采集 USB 巡线相机的放置白横线和 LineTrack 黄金统计。"""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rk_interfaces.msg import LineTrack, SpecialTargetDetection

from capture_pickup_board_calibration import _summary


class PlaceWhiteBarCapture(Node):
    """在手工摆正机器人后记录白线纵向与黑线横向/航向误差。"""

    def __init__(self, white_topic, line_topic):
        super().__init__('capture_place_white_bar_calibration')
        self.samples = {
            name: [] for name in (
                'center_y_ratio',
                'span_ratio',
                'height_ratio',
                'confidence',
                'line_lateral_error',
                'line_heading_error')}
        self.create_subscription(
            SpecialTargetDetection,
            white_topic,
            self._on_white,
            10)
        self.create_subscription(LineTrack, line_topic, self._on_line, 10)

    def _on_white(self, message):
        if message.visible:
            self.samples['center_y_ratio'].append(message.center_y)
            self.samples['span_ratio'].append(message.width_ratio)
            self.samples['height_ratio'].append(message.height_ratio)
            self.samples['confidence'].append(message.confidence)

    def _on_line(self, message):
        if message.line_visible:
            self.samples['line_lateral_error'].append(message.lateral_error)
            self.samples['line_heading_error'].append(message.heading_error)


def main():
    """支持 place1/place2 标签，输出仅作为审核证据。"""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--white-topic',
        default='/perception/white_bar_detection')
    parser.add_argument('--line-topic', default='/perception/line_track')
    parser.add_argument(
        '--platform',
        choices=(
            'place1',
            'place2'),
        required=True)
    parser.add_argument('--duration-sec', type=float, default=5.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.duration_sec <= 0.0:
        raise SystemExit('--duration-sec must be positive')
    output = args.output or '{}_WHITE_BAR_GOLDEN_SIGNATURE.json'.format(
        args.platform.upper())
    rclpy.init()
    node = PlaceWhiteBarCapture(args.white_topic, args.line_topic)
    deadline = time.monotonic() + args.duration_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(
                    0.2, deadline - time.monotonic()))
        payload = {
            'schema': 'place_white_bar_golden_signature/v1',
            'platform': args.platform,
            'calibration_required': True,
            'duration_sec': args.duration_sec,
            'metrics': {
                key: _summary(values) for key,
                values in node.samples.items()}}
        Path(output).write_text(
            json.dumps(
                payload,
                indent=2,
                sort_keys=True) +
            '\n',
            encoding='utf-8')
        print('Wrote {}'.format(output))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
