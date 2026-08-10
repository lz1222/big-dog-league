#!/usr/bin/env python3

"""D1-T 固定关节轨迹的离线预览工具（绝不发送机械臂命令）。

位置一与位置二使用操作者提供的 App 显示单位。程序将两点按最大关节步长
做线性插补，供现场核对关节映射、机械干涉和预计时间。它没有 ROS、DDS、
串口或网络写入功能，因此不能移动 D1-T。

用法：
    python3 tools/d1_fixed_pose_move.py
    python3 tools/d1_fixed_pose_move.py --max-joint-step-deg 3 --interval-sec 0.30
"""

import argparse
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


# App 关节 1~6 与 SDK angle0~angle5 的映射已在项目中记录为现场确认映射。
# 第七路 angle6 为夹爪；其 0/40 的安全范围尚未由厂家协议正式确认，故仅展示。
START_POSE = (0.0, -90.0, 90.0, 0.0, 0.0, 0.0, 0.0)
TARGET_POSE = (-86.0, 50.0, 41.0, 13.0, -79.0, -1.0, 40.0)

CHANNEL_NAMES = (
    '关节1(angle0)', '关节2(angle1)', '关节3(angle2)',
    '关节4(angle3)', '关节5(angle4)', '关节6(angle5)', '爪夹(angle6)',
)

# 来自用户提供的 D1-T 关节行程图。夹爪为 App 显示值，当前不伪造机械范围。
JOINT_LIMITS_DEG = (
    (-90.0, 90.0),
    (-90.0, 90.0),
    (-135.0, 135.0),
    (-90.0, 90.0),
    (-135.0, 135.0),
    (-40.0, 20.0),
)


@dataclass(frozen=True)
class TrajectoryPlan:
    """已验证的七通道离线轨迹；每行是一个完整目标姿态，不代表发送命令。"""

    samples: Tuple[Tuple[float, ...], ...]
    interval_sec: float


def _finite_pose(pose: Sequence[float], name: str) -> Tuple[float, ...]:
    """拒绝非有限值或缺通道，防止 NaN/截断数据变成错误关节目标。"""
    if len(pose) != len(CHANNEL_NAMES):
        raise ValueError('{0} must contain exactly 7 channels'.format(name))
    values = tuple(float(value) for value in pose)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('{0} contains a non-finite value'.format(name))
    return values


def validate_joint_limits(pose: Sequence[float], name: str) -> None:
    """只按已知 D1-T 六关节行程检查；夹爪范围未知时明确提示而不假定安全。"""
    values = _finite_pose(pose, name)
    for index, (minimum, maximum) in enumerate(JOINT_LIMITS_DEG):
        value = values[index]
        if value < minimum or value > maximum:
            raise ValueError(
                '{0} {1}={2:.3f} exceeds known limit [{3:.1f}, {4:.1f}]'.format(
                    name, CHANNEL_NAMES[index], value, minimum, maximum))


def build_linear_plan(start: Sequence[float], target: Sequence[float],
                      max_joint_step_deg: float,
                      interval_sec: float) -> TrajectoryPlan:
    """以完整七通道插补生成轨迹，避免将起点直接跳变为终点。

    所有关节和夹爪在同一采样时刻同步插补。最大步长只约束六个机械关节；
    夹爪的 0→40 行程保留在完整姿态中，但不能据此推断其物理速度或力。
    """
    start_values = _finite_pose(start, 'start_pose')
    target_values = _finite_pose(target, 'target_pose')
    validate_joint_limits(start_values, 'start_pose')
    validate_joint_limits(target_values, 'target_pose')
    if not math.isfinite(max_joint_step_deg) or max_joint_step_deg <= 0.0:
        raise ValueError('max_joint_step_deg must be a positive finite value')
    if not math.isfinite(interval_sec) or interval_sec <= 0.0:
        raise ValueError('interval_sec must be a positive finite value')

    # 只以六关节最大角位移决定样本数，确保每一机械关节增量不超过上限。
    maximum_delta = max(
        abs(target_values[index] - start_values[index]) for index in range(6))
    steps = max(1, int(math.ceil(maximum_delta / max_joint_step_deg)))
    samples: List[Tuple[float, ...]] = []
    for step in range(steps + 1):
        fraction = float(step) / float(steps)
        samples.append(tuple(
            start_values[index] +
            (target_values[index] - start_values[index]) * fraction
            for index in range(len(CHANNEL_NAMES))))
    return TrajectoryPlan(tuple(samples), float(interval_sec))


def _format_pose(values: Iterable[float]) -> str:
    """使用固定通道顺序打印，避免现场将 App 关节编号与 SDK angle 编号混淆。"""
    return '  '.join(
        '{0}={1:7.2f}'.format(name, value)
        for name, value in zip(CHANNEL_NAMES, values))


def parse_args() -> argparse.Namespace:
    """提供保守插补参数；此工具不提供 execute 开关以保证它始终离线。"""
    parser = argparse.ArgumentParser(
        description='Preview the fixed D1-T pose-one to pose-two trajectory (dry run only).')
    parser.add_argument('--max-joint-step-deg', type=float, default=5.0,
                        help='Maximum increment per mechanical joint (default: 5 degrees).')
    parser.add_argument('--interval-sec', type=float, default=0.25,
                        help='Planned interval between samples (default: 0.25 seconds).')
    return parser.parse_args()


def main() -> int:
    """打印可审计轨迹。真机控制接口未确认时，任何输出都不得直接发送。"""
    args = parse_args()
    try:
        plan = build_linear_plan(
            START_POSE, TARGET_POSE,
            args.max_joint_step_deg, args.interval_sec)
    except ValueError as error:
        print('轨迹拒绝：{0}'.format(error))
        return 2

    print('D1-T 固定轨迹预览（DRY RUN ONLY / NOT SENT）')
    print('起点：{0}'.format(_format_pose(START_POSE)))
    print('终点：{0}'.format(_format_pose(TARGET_POSE)))
    print('共 {0} 个姿态点，点间规划间隔 {1:.2f}s，预计 {2:.2f}s。'.format(
        len(plan.samples), plan.interval_sec,
        (len(plan.samples) - 1) * plan.interval_sec))
    print('注意：爪夹 angle6=0→40 的厂家行程、速度、力与停止协议尚未确认。')
    for index, pose in enumerate(plan.samples):
        print('第 {0:02d} 点：{1}'.format(index, _format_pose(pose)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
