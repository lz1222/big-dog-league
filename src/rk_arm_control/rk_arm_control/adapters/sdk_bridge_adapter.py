#!/usr/bin/env python3

import json
import threading
import time
from typing import Dict, List

from rclpy.node import Node
from std_msgs.msg import String

from rk_arm_control.adapters.base import ArmHardwareAdapter


class SdkBridgeArmAdapter(ArmHardwareAdapter):
    """SDK 桥接适配器。

    这个适配器不直接 import 某个机械臂厂家的 SDK，而是把上层动作转换成
    JSON 后发布到一个桥接 topic。后续新机械臂到了，可以单独写一个
    bridge 进程订阅这个 topic，再调用厂家 Python/C++/串口/CAN SDK。

    参考宇树 D1 SDK：
    - `joint_angle_control.cpp` 使用 `funcode=1` 单关节命令；
    - `multiple_joint_angle_control.cpp` 使用 `funcode=2` 多关节命令；
    - D1 示例通过 `rt/arm_Command` 发送 JSON 字符串。

    本适配器默认使用 `generic_json`，不假设新机械臂协议。若需要对照 D1
    调试，可以把 YAML 中 `command_format` 改为
    `unitree_d1_json_reference`，但真实新机械臂仍建议单独实现 bridge。
    """

    def __init__(self, node: Node, config: Dict):
        self._node = node
        self._logger = node.get_logger()
        self._config = dict(config or {})
        self._command_topic = str(
            self._config.get('command_topic', '/arm/sdk_bridge/command_json')
        )
        self._command_format = str(
            self._config.get('command_format', 'generic_json')
        )
        self._joint_unit = str(self._config.get('joint_unit', 'deg'))
        self._fire_and_wait = bool(self._config.get('fire_and_wait', True))
        self._seq = int(self._config.get('initial_seq', 1000))
        self._stop_event = threading.Event()
        self._publisher = node.create_publisher(String, self._command_topic, 10)

    def initialize(self) -> bool:
        self._logger.warn(
            'new arm adapter mode=sdk_bridge: 已启用桥接输出。'
            f' command_topic={self._command_topic}, '
            f'format={self._command_format}'
        )
        return True

    def move_joints(
        self,
        joints: List[float],
        duration_sec: float,
        pose_name: str = '',
    ) -> bool:
        self._stop_event.clear()
        payload = self._build_move_payload(joints, duration_sec, pose_name)
        self._publish_payload(payload)
        if self._fire_and_wait:
            return not self._sleep(duration_sec)
        return not self._stop_event.is_set()

    def open_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        payload = self._build_gripper_payload('open', duration_sec)
        self._publish_payload(payload)
        if self._fire_and_wait:
            return not self._sleep(duration_sec)
        return not self._stop_event.is_set()

    def close_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        payload = self._build_gripper_payload('close', duration_sec)
        self._publish_payload(payload)
        if self._fire_and_wait:
            return not self._sleep(duration_sec)
        return not self._stop_event.is_set()

    def stop(self) -> None:
        self._stop_event.set()
        payload = self._build_stop_payload()
        self._publish_payload(payload)
        self._logger.warn('SDK bridge stop requested')

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _build_move_payload(
        self,
        joints: List[float],
        duration_sec: float,
        pose_name: str,
    ) -> Dict:
        if self._command_format == 'unitree_d1_json_reference':
            data = {
                'mode': 1,
            }
            for index, value in enumerate(joints[:7]):
                data[f'angle{index}'] = float(value)
            return {
                'seq': self._next_seq(),
                'address': 1,
                'funcode': 2,
                'data': data,
                'duration_sec': float(duration_sec),
                'pose_name': pose_name,
            }

        # 通用 JSON：建议新机械臂 bridge 优先支持这个格式。
        return {
            'seq': self._next_seq(),
            'command': 'MOVE_JOINTS',
            'pose_name': pose_name,
            'joints': [float(value) for value in joints],
            'joint_unit': self._joint_unit,
            'duration_sec': float(duration_sec),
        }

    def _build_gripper_payload(self, action: str, duration_sec: float) -> Dict:
        return {
            'seq': self._next_seq(),
            'command': 'GRIPPER',
            'action': action,
            'duration_sec': float(duration_sec),
        }

    def _build_stop_payload(self) -> Dict:
        return {
            'seq': self._next_seq(),
            'command': 'STOP',
            'reason': 'arm task stop requested',
        }

    def _publish_payload(self, payload: Dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self._publisher.publish(msg)
        self._logger.info(
            f'publish SDK bridge command: topic={self._command_topic} '
            f'payload={msg.data}'
        )

    def _sleep(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(min(0.02, deadline - time.monotonic()))
        return self._stop_event.is_set()

