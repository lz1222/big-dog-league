"""既有 D1 固定放置程序的 ExecuteArmTask 适配器。

本模块只选择和执行已经编译的固定动作程序，不拥有任何关节值、夹爪值或
轨迹。默认 ``enabled=false``，使未完成实体许可时的 Action 请求安全失败。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Callable, Mapping, Optional, Sequence


FIXED_PLATFORM_EXECUTABLES = {
    'PLACE_PLATFORM_1': 'd1_platform1_complete_action',
    'PLACE_PLATFORM_2': 'd1_platform2_complete_action',
}


@dataclass(frozen=True)
class FixedPlatformActionResult:
    """固定动作进程的明确结果，非零退出和超时均不能报告成功。"""

    success: bool
    message: str


class FixedPlatformActionRunner:
    """以无 shell 的子进程调用现有 D1 固定放置 executable。"""

    def __init__(
            self,
            config: Optional[Mapping[str, object]] = None,
            executor: Callable[..., object] = subprocess.run):
        config = dict(config or {})
        self.enabled = bool(config.get('enabled', False))
        self.binary_directory = Path(str(config.get(
            'binary_directory',
            '/home/unitree/rk_inspection_ws/build/unitree_d1_sdk')))
        self.network_interface = str(config.get('network_interface', 'eth1'))
        self.timeout_sec = max(1.0, float(config.get('timeout_sec', 90.0)))
        self._executor = executor

    @staticmethod
    def supports(task_name: str) -> bool:
        """只接受两项固定放置任务，避免 mission 借此执行任意程序。"""
        return str(task_name or '').upper() in FIXED_PLATFORM_EXECUTABLES

    def execute(self, task_name: str) -> FixedPlatformActionResult:
        """运行一项已审计的固定动作；未授权、缺文件和异常全部失败关闭。"""
        normalized = str(task_name or '').upper()
        executable_name = FIXED_PLATFORM_EXECUTABLES.get(normalized)
        if executable_name is None:
            return FixedPlatformActionResult(False, 'unsupported fixed task')
        if not self.enabled:
            return FixedPlatformActionResult(
                False, 'fixed platform actions are disabled')
        executable = self.binary_directory / executable_name
        if not executable.is_file() or not executable.stat().st_mode & 0o111:
            return FixedPlatformActionResult(
                False,
                'fixed action executable unavailable: {}'.format(executable))
        command: Sequence[str] = (
            str(executable), self.network_interface, '--execute', '--power-on')
        try:
            completed = self._executor(
                command, check=False, capture_output=True, text=True,
                timeout=self.timeout_sec)
        except subprocess.TimeoutExpired:
            return FixedPlatformActionResult(False, 'fixed action timeout')
        except OSError as error:
            return FixedPlatformActionResult(
                False, 'fixed action launch failed: {}'.format(error))
        if int(getattr(completed, 'returncode', 1)) != 0:
            return FixedPlatformActionResult(
                False, 'fixed action failed: {}'.format(
                    str(getattr(completed, 'stderr', '')).strip()))
        return FixedPlatformActionResult(
            True, 'fixed action completed: {}'.format(executable_name))
