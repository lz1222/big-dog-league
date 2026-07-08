#!/usr/bin/env python3

import threading
import time
from typing import List

from rclpy.node import Node

from rk_arm_control.adapters.base import ArmHardwareAdapter


class DryRunArmAdapter(ArmHardwareAdapter):
    """空跑适配器。

    用途：
    1. 新机械臂没到时先验证任务流程、状态发布和超时逻辑；
    2. 上真机前先确认 YAML 点位、任务顺序和命令入口；
    3. 避免未知 SDK 命令直接误动硬件。
    """

    def __init__(self, node: Node):
        self._logger = node.get_logger()
        self._stop_event = threading.Event()

    def initialize(self) -> bool:
        self._logger.warn(
            'new arm adapter mode=dry_run: 只打印动作，不控制真实机械臂。'
        )
        return True

    def move_joints(
        self,
        joints: List[float],
        duration_sec: float,
        pose_name: str = '',
    ) -> bool:
        self._stop_event.clear()
        self._logger.info(
            '[DRY_RUN] move_joints '
            f'pose={pose_name or "unnamed"} joints={joints} '
            f'duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def open_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] open_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def close_gripper(self, duration_sec: float) -> bool:
        self._stop_event.clear()
        self._logger.info(
            f'[DRY_RUN] close_gripper duration_sec={duration_sec:.2f}'
        )
        return not self._sleep(duration_sec)

    def stop(self) -> None:
        self._stop_event.set()
        self._logger.warn('[DRY_RUN] stop requested')

    def _sleep(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(duration_sec))
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return True
            time.sleep(min(0.02, deadline - time.monotonic()))
        return self._stop_event.is_set()

