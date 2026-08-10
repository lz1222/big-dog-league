"""完整 D1 放置任务的编排合同测试；不启动 ROS 或真实机械臂。"""

from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace

import yaml

from rk_arm_control.fixed_platform_action_runner import (
    FIXED_PLATFORM_EXECUTABLES,
    FixedPlatformActionRunner,
    fixed_platform_task_plan_error,
)
from rk_arm_control.new_arm_task_node import (
    ExecutionResult,
    NewArmTaskNode,
    STATUS_FAILED,
    STATUS_RUNNING,
)


POSES_FILE = Path(__file__).parents[1] / 'config' / 'new_arm_poses.yaml'


def _platform_steps(task_name):
    """读取正式 YAML，避免测试只覆盖与生产脱节的手工副本。"""
    with POSES_FILE.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    return config['new_arm']['tasks'][task_name]['steps']


def test_platform_tasks_are_exactly_one_complete_fixed_action():
    """一号和二号都只能调各自的完整 executable，不能携带 legacy step。"""
    for task_name, executable in FIXED_PLATFORM_EXECUTABLES.items():
        steps = _platform_steps(task_name)
        assert steps == [{'type': 'fixed_platform_action'}]
        assert not fixed_platform_task_plan_error(task_name, steps)
        assert executable in (
            'd1_platform1_complete_action',
            'd1_platform2_complete_action',
        )


def test_platform_plan_rejects_legacy_trailing_steps_before_execution():
    """错误 YAML 必须在启动 D1 子进程前失败，不能 fallback 到通用动作。"""
    legacy_plan = [
        {'type': 'fixed_platform_action'},
        {'type': 'gripper', 'action': 'open'},
    ]
    assert (fixed_platform_task_plan_error('PLACE_PLATFORM_2', legacy_plan)
            == 'FIXED_PLATFORM_TASK_HAS_TRAILING_STEPS')


class _SequenceHarness:
    """用最小状态机替身验证 action 序列，不创建 ROS node 或硬件进程。"""

    def __init__(self, steps, step_result):
        self.tasks = {'PLACE_PLATFORM_2': {'steps': steps}}
        self._active_task = ''
        self._current_step = ''
        self._abort_event = threading.Event()
        self._state_lock = threading.RLock()
        self._step_result = step_result
        self.executed_steps = []

    def _try_begin_task(self, task_name):
        self._active_task = task_name
        return True

    def _should_abort(self, goal_handle):
        del goal_handle
        return False

    def _step_name(self, step):
        return str(step.get('type', '')).upper()

    def _publish_action_feedback(self, goal_handle, step, progress):
        del goal_handle, step, progress

    def publish_status(self, task, state, step, success, message):
        del task, state, step, success, message

    def publish_lock(self, locked):
        del locked

    def _execute_step(self, step, task_name, step_name):
        del task_name, step_name
        self.executed_steps.append(step)
        return self._step_result


def test_action_sequence_maps_fixed_result_and_never_runs_legacy_steps():
    """固定步骤成功/失败返回 action 结果；错误尾随计划在执行前拒绝。"""
    fixed_step = [{'type': 'fixed_platform_action'}]
    success_harness = _SequenceHarness(
        fixed_step, ExecutionResult(True, STATUS_RUNNING, 'fixed success'))
    success = NewArmTaskNode.execute_task_sequence(
        success_harness, 'PLACE_PLATFORM_2')
    assert success.success
    assert success_harness.executed_steps == fixed_step

    failure_harness = _SequenceHarness(
        fixed_step, ExecutionResult(False, STATUS_FAILED, 'fixed failure'))
    failure = NewArmTaskNode.execute_task_sequence(
        failure_harness, 'PLACE_PLATFORM_2')
    assert not failure.success
    assert failure_harness.executed_steps == fixed_step

    trailing_step = fixed_step + [{'type': 'move', 'pose': 'HOME'}]
    rejected_harness = _SequenceHarness(
        trailing_step, ExecutionResult(True, STATUS_RUNNING, 'unexpected'))
    rejected = NewArmTaskNode.execute_task_sequence(
        rejected_harness, 'PLACE_PLATFORM_2')
    assert not rejected.success
    assert rejected.message == 'FIXED_PLATFORM_TASK_HAS_TRAILING_STEPS'
    assert rejected_harness.executed_steps == []


def test_fixed_runner_success_failure_and_timeout_do_not_fallback(tmp_path):
    """runner 的成功、失败和超时均只调用一次指定 executable。"""
    executable = Path(tmp_path) / 'd1_platform2_complete_action'
    executable.touch()
    executable.chmod(0o755)
    commands = []

    def success(command, **kwargs):
        commands.append((command, kwargs))
        return SimpleNamespace(returncode=0, stderr='')

    runner = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path)}, executor=success)
    assert runner.execute('PLACE_PLATFORM_2').success
    assert len(commands) == 1
    assert commands[0][0][0].endswith('d1_platform2_complete_action')

    def failure(*args, **kwargs):
        del args, kwargs
        return SimpleNamespace(returncode=1, stderr='failed')

    failed = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path)},
        executor=failure)
    assert not failed.execute('PLACE_PLATFORM_2').success

    timed_out = FixedPlatformActionRunner(
        {'enabled': True, 'binary_directory': str(tmp_path)},
        executor=lambda *args, **kwargs: (_ for _ in ()).throw(
            subprocess.TimeoutExpired(args[0], kwargs['timeout'])))
    assert not timed_out.execute('PLACE_PLATFORM_2').success
