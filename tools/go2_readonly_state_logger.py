#!/usr/bin/env python3
"""
Go2 只读状态数据采集工具

READ-ONLY MODE: NO MOTION COMMANDS WILL BE SENT

本工具仅订阅 /lowstate 和 /sportmodestate，不发布任何运动控制话题。
用于诊断左后腿异常发热和停车左偏问题。

用法:
  python3 go2_readonly_state_logger.py --duration 30 --rate 2
  python3 go2_readonly_state_logger.py --duration 60 --rate 5 --output /tmp/diagnosis

输出:
  <output>_YYYYMMDD_HHMMSS.csv
  <output>_YYYYMMDD_HHMMSS.json
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

# 电机索引常量（来自 Unitree SDK motor_crc.h）
MOTOR_INDEX = {
    # 右前腿
    'FR_hip': 0, 'FR_thigh': 1, 'FR_calf': 2,
    # 左前腿
    'FL_hip': 3, 'FL_thigh': 4, 'FL_calf': 5,
    # 右后腿
    'RR_hip': 6, 'RR_thigh': 7, 'RR_calf': 8,
    # 左后腿
    'RL_hip': 9, 'RL_thigh': 10, 'RL_calf': 11,
}

# 需要对比的左右后腿关节对
REAR_LEG_PAIRS = [
    ('RR_hip', 'RL_hip', 6, 9),
    ('RR_thigh', 'RL_thigh', 7, 10),
    ('RR_calf', 'RL_calf', 8, 11),
]

# 前腿也记录但不作为主要对比
FRONT_LEG_INDICES = {
    'FR_hip': 0, 'FR_thigh': 1, 'FR_calf': 2,
    'FL_hip': 3, 'FL_thigh': 4, 'FL_calf': 5,
}

# 足端力顺序（Go2: FR=0, FL=1, RR=2, RL=3）
FOOT_FORCE_INDEX = {'FR': 0, 'FL': 1, 'RR': 2, 'RL': 3}


def make_fieldnames():
    """生成 CSV 列名"""
    fields = ['timestamp', 'elapsed_sec']

    # 所有 12 个关节的 temperature
    for leg in ['FR', 'FL', 'RR', 'RL']:
        for joint in ['hip', 'thigh', 'calf']:
            fields.append(f'{leg}_{joint}_temp')

    # 所有 12 个关节的 tau_est
    for leg in ['FR', 'FL', 'RR', 'RL']:
        for joint in ['hip', 'thigh', 'calf']:
            fields.append(f'{leg}_{joint}_tau_est')

    # 所有 12 个关节的 q
    for leg in ['FR', 'FL', 'RR', 'RL']:
        for joint in ['hip', 'thigh', 'calf']:
            fields.append(f'{leg}_{joint}_q')

    # 所有 12 个关节的 dq
    for leg in ['FR', 'FL', 'RR', 'RL']:
        for joint in ['hip', 'thigh', 'calf']:
            fields.append(f'{leg}_{joint}_dq')

    # 足端力
    for foot in ['FR', 'FL', 'RR', 'RL']:
        fields.append(f'{foot}_foot_force')
        fields.append(f'{foot}_foot_force_est')

    # IMU
    fields.append('imu_roll')
    fields.append('imu_pitch')
    fields.append('imu_yaw')

    # 运动状态
    fields.append('sport_mode')
    fields.append('gait_type')
    fields.append('progress')
    fields.append('body_height')

    return fields


def extract_motor_data(lowstate, motor_idx):
    """从 LowState 提取单个电机的数据"""
    try:
        motor = lowstate.motor_state[motor_idx]
        return {
            'temp': motor.temperature,
            'tau_est': motor.tau_est,
            'q': motor.q,
            'dq': motor.dq,
        }
    except IndexError:
        return {'temp': None, 'tau_est': None, 'q': None, 'dq': None}


def extract_row(lowstate, sport_state, elapsed_sec):
    """从 LowState 和 SportModeState 提取一行数据"""
    row = {
        'timestamp': datetime.now().isoformat(),
        'elapsed_sec': round(elapsed_sec, 3),
    }

    # 提取所有电机数据
    all_legs = [
        ('FR', 'hip', 0), ('FR', 'thigh', 1), ('FR', 'calf', 2),
        ('FL', 'hip', 3), ('FL', 'thigh', 4), ('FL', 'calf', 5),
        ('RR', 'hip', 6), ('RR', 'thigh', 7), ('RR', 'calf', 8),
        ('RL', 'hip', 9), ('RL', 'thigh', 10), ('RL', 'calf', 11),
    ]
    for leg, joint, idx in all_legs:
        data = extract_motor_data(lowstate, idx)
        row[f'{leg}_{joint}_temp'] = data['temp']
        row[f'{leg}_{joint}_tau_est'] = data['tau_est']
        row[f'{leg}_{joint}_q'] = data['q']
        row[f'{leg}_{joint}_dq'] = data['dq']

    # 足端力（LowState 中的更精确）
    for foot, fi in FOOT_FORCE_INDEX.items():
        try:
            row[f'{foot}_foot_force'] = lowstate.foot_force[fi]
        except IndexError:
            row[f'{foot}_foot_force'] = None
        try:
            row[f'{foot}_foot_force_est'] = lowstate.foot_force_est[fi]
        except IndexError:
            row[f'{foot}_foot_force_est'] = None

    # IMU 数据（使用 LowState 中的 IMU）
    try:
        row['imu_roll'] = lowstate.imu_state.rpy[0]
        row['imu_pitch'] = lowstate.imu_state.rpy[1]
        row['imu_yaw'] = lowstate.imu_state.rpy[2]
    except (AttributeError, IndexError):
        row['imu_roll'] = None
        row['imu_pitch'] = None
        row['imu_yaw'] = None

    # 运动状态（来自 SportModeState）
    if sport_state is not None:
        row['sport_mode'] = sport_state.mode
        row['gait_type'] = sport_state.gait_type
        row['progress'] = sport_state.progress
        row['body_height'] = sport_state.body_height
    else:
        row['sport_mode'] = None
        row['gait_type'] = None
        row['progress'] = None
        row['body_height'] = None

    return row


class Go2ReadonlyStateLogger(Node):
    """只读订阅 Go2 低层状态，定期记录为 CSV/JSON。"""

    def __init__(self, duration_sec, rate_hz, output_prefix):
        super().__init__('go2_readonly_state_logger')

        self.duration_sec = duration_sec
        self.rate_hz = rate_hz
        self.output_prefix = output_prefix
        self.rows = []
        self.start_time = None
        self.latest_lowstate = None
        self.latest_sport_state = None
        self.lowstate_count = 0
        self.sport_state_count = 0
        self._shutting_down = False

        # 订阅 /lowstate
        try:
            from unitree_go.msg import LowState
            self.lowstate_sub = self.create_subscription(
                LowState,
                '/lowstate',
                self._on_lowstate,
                10
            )
            self.get_logger().info('Subscribed to /lowstate')
        except Exception as e:
            self.get_logger().error(f'Failed to subscribe to /lowstate: {e}')
            self.lowstate_sub = None

        # 订阅 /sportmodestate
        try:
            from unitree_go.msg import SportModeState
            self.sport_state_sub = self.create_subscription(
                SportModeState,
                '/sportmodestate',
                self._on_sport_state,
                10
            )
            self.get_logger().info('Subscribed to /sportmodestate')
        except Exception as e:
            self.get_logger().error(
                f'Failed to subscribe to /sportmodestate: {e}'
            )
            self.sport_state_sub = None

        # 记录定时器
        self.record_period = 1.0 / rate_hz
        self.record_timer = self.create_timer(
            self.record_period,
            self._on_record_timer
        )

        self.start_time = time.monotonic()
        self.get_logger().info(
            '============================================================'
        )
        self.get_logger().info(
            'READ-ONLY MODE: NO MOTION COMMANDS WILL BE SENT'
        )
        self.get_logger().info(
            f'Duration: {duration_sec}s, Rate: {rate_hz}Hz'
        )
        self.get_logger().info(
            '============================================================'
        )

    def _on_lowstate(self, msg):
        self.lowstate_count += 1
        self.latest_lowstate = msg

    def _on_sport_state(self, msg):
        self.sport_state_count += 1
        self.latest_sport_state = msg

    def _on_record_timer(self):
        if self._shutting_down:
            return

        elapsed = time.monotonic() - self.start_time
        if elapsed > self.duration_sec:
            self._shutdown()
            return

        if self.latest_lowstate is None:
            self.get_logger().warn(
                f'T+{elapsed:.1f}s: waiting for /lowstate...',
                throttle_duration_sec=5.0
            )
            return

        row = extract_row(
            self.latest_lowstate,
            self.latest_sport_state,
            elapsed
        )
        self.rows.append(row)

        # 实时简要输出
        self._print_live_summary(row, elapsed)

    def _print_live_summary(self, row, elapsed):
        """实时输出左右后腿对比摘要"""
        rl_temp = [
            row.get('RL_hip_temp', '?'),
            row.get('RL_thigh_temp', '?'),
            row.get('RL_calf_temp', '?'),
        ]
        rr_temp = [
            row.get('RR_hip_temp', '?'),
            row.get('RR_thigh_temp', '?'),
            row.get('RR_calf_temp', '?'),
        ]
        rl_tau = [
            row.get('RL_hip_tau_est', '?'),
            row.get('RL_thigh_tau_est', '?'),
            row.get('RL_calf_tau_est', '?'),
        ]
        rr_tau = [
            row.get('RR_hip_tau_est', '?'),
            row.get('RR_thigh_tau_est', '?'),
            row.get('RR_calf_tau_est', '?'),
        ]

        temp_diff_str = ''
        for i, joint in enumerate(['hip', 'thigh', 'calf']):
            if isinstance(rl_temp[i], (int, float)) and isinstance(rr_temp[i], (int, float)):
                diff = rl_temp[i] - rr_temp[i]
                temp_diff_str += f'{joint}:RL={rl_temp[i]}°C/RR={rr_temp[i]}°C/Δ={diff} '

        tau_diff_str = ''
        for i, joint in enumerate(['hip', 'thigh', 'calf']):
            if isinstance(rl_tau[i], (int, float)) and isinstance(rr_tau[i], (int, float)):
                diff = rl_tau[i] - rr_tau[i]
                tau_diff_str += f'{joint}:RL={rl_tau[i]:.2f}/RR={rr_tau[i]:.2f}/Δ={diff:.2f} '

        self.get_logger().info(
            f'T+{elapsed:.1f}s | TEMP: {temp_diff_str}| TAU: {tau_diff_str}'
        )

    def _shutdown(self):
        if self._shutting_down:
            return
        self._shutting_down = True

        self.get_logger().info(
            f'Collection complete. '
            f'lowstate frames: {self.lowstate_count}, '
            f'sport_state frames: {self.sport_state_count}, '
            f'records: {len(self.rows)}'
        )

        if not self.rows:
            self.get_logger().error('No data collected!')
            return

        self._save_output()
        self._print_analysis()

    def _save_output(self):
        """保存 CSV 和 JSON 文件"""
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = f'{self.output_prefix}_{ts}.csv'
        json_path = f'{self.output_prefix}_{ts}.json'

        fieldnames = make_fieldnames()

        # CSV
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames,
                                        extrasaction='ignore')
                writer.writeheader()
                writer.writerows(self.rows)
            self.get_logger().info(f'CSV saved to: {csv_path}')
        except OSError as e:
            self.get_logger().error(f'Failed to write CSV: {e}')

        # JSON
        try:
            with open(json_path, 'w') as f:
                json.dump(self.rows, f, indent=2, default=str)
            self.get_logger().info(f'JSON saved to: {json_path}')
        except OSError as e:
            self.get_logger().error(f'Failed to write JSON: {e}')

        print(f'\nOUTPUT_CSV={csv_path}')
        print(f'OUTPUT_JSON={json_path}\n')

    def _print_analysis(self):
        """打印快速分析"""
        print('\n========== QUICK ANALYSIS ==========')

        # 温度分析
        print('\n--- 温度对比 (°C) ---')
        for rr_name, rl_name, rr_idx, rl_idx in REAR_LEG_PAIRS:
            rr_temps = [r[f'{rr_name}_temp'] for r in self.rows
                        if r.get(f'{rr_name}_temp') is not None]
            rl_temps = [r[f'{rl_name}_temp'] for r in self.rows
                        if r.get(f'{rl_name}_temp') is not None]

            if rr_temps and rl_temps:
                rr_avg = sum(rr_temps) / len(rr_temps)
                rl_avg = sum(rl_temps) / len(rl_temps)
                rr_max = max(rr_temps)
                rl_max = max(rl_temps)
                print(f'  {rr_name}: avg={rr_avg:.1f}, max={rr_max}')
                print(f'  {rl_name}: avg={rl_avg:.1f}, max={rl_max}')
                print(f'  温差: Δavg={rl_avg - rr_avg:.1f}, Δmax={rl_max - rr_max}')

                # 温升速率
                if len(rl_temps) >= 2:
                    duration = self.rows[-1]['elapsed_sec'] - self.rows[0]['elapsed_sec']
                    if duration > 0:
                        rl_rate = (rl_temps[-1] - rl_temps[0]) / (duration / 60.0)
                        rr_rate = (rr_temps[-1] - rr_temps[0]) / (duration / 60.0)
                        print(f'  温升速率: RL={rl_rate:.1f}°C/min, RR={rr_rate:.1f}°C/min')

        # 力矩分析
        print('\n--- 估计力矩对比 (tau_est, N·m) ---')
        for rr_name, rl_name, rr_idx, rl_idx in REAR_LEG_PAIRS:
            rr_tau = [r[f'{rr_name}_tau_est'] for r in self.rows
                      if r.get(f'{rr_name}_tau_est') is not None]
            rl_tau = [r[f'{rl_name}_tau_est'] for r in self.rows
                      if r.get(f'{rl_name}_tau_est') is not None]

            if rr_tau and rl_tau:
                rr_avg = sum(rr_tau) / len(rr_tau)
                rl_avg = sum(rl_tau) / len(rl_tau)
                rr_std = math.sqrt(
                    sum((x - rr_avg) ** 2 for x in rr_tau) / len(rr_tau)
                ) if len(rr_tau) > 1 else 0
                rl_std = math.sqrt(
                    sum((x - rl_avg) ** 2 for x in rl_tau) / len(rl_tau)
                ) if len(rl_tau) > 1 else 0
                print(f'  {rr_name}: avg={rr_avg:.3f}, std={rr_std:.3f}')
                print(f'  {rl_name}: avg={rl_avg:.3f}, std={rl_std:.3f}')
                print(f'  力矩差: Δavg={rl_avg - rr_avg:.3f}')

        # 足端力分析
        print('\n--- 足端力分布 ---')
        rl_forces = [r.get('RL_foot_force') for r in self.rows
                     if r.get('RL_foot_force') is not None]
        rr_forces = [r.get('RR_foot_force') for r in self.rows
                     if r.get('RR_foot_force') is not None]
        fl_forces = [r.get('FL_foot_force') for r in self.rows
                     if r.get('FL_foot_force') is not None]
        fr_forces = [r.get('FR_foot_force') for r in self.rows
                     if r.get('FR_foot_force') is not None]

        if all([rl_forces, rr_forces, fl_forces, fr_forces]):
            rl_avg = sum(rl_forces) / len(rl_forces)
            rr_avg = sum(rr_forces) / len(rr_forces)
            fl_avg = sum(fl_forces) / len(fl_forces)
            fr_avg = sum(fr_forces) / len(fr_forces)
            total = rl_avg + rr_avg + fl_avg + fr_avg
            if total > 0:
                print(f'  FR: {fr_avg:.1f} ({fr_avg/total*100:.1f}%)')
                print(f'  FL: {fl_avg:.1f} ({fl_avg/total*100:.1f}%)')
                print(f'  RR: {rr_avg:.1f} ({rr_avg/total*100:.1f}%)')
                print(f'  RL: {rl_avg:.1f} ({rl_avg/total*100:.1f}%)')
                print(f'  左右后腿力差: RL-RR = {rl_avg - rr_avg:.1f}')

        # 姿态
        print('\n--- 机身姿态 (rad) ---')
        rolls = [r.get('imu_roll') for r in self.rows
                 if r.get('imu_roll') is not None]
        pitchs = [r.get('imu_pitch') for r in self.rows
                  if r.get('imu_pitch') is not None]
        if rolls and pitchs:
            print(f'  roll  avg={sum(rolls)/len(rolls):.4f} rad '
                  f'({math.degrees(sum(rolls)/len(rolls)):.2f}°)')
            print(f'  pitch avg={sum(pitchs)/len(pitchs):.4f} rad '
                  f'({math.degrees(sum(pitchs)/len(pitchs)):.2f}°)')

        print('\n=====================================\n')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Go2 Read-Only State Logger'
    )
    parser.add_argument(
        '--duration', type=float, default=30.0,
        help='采集时长（秒），默认 30'
    )
    parser.add_argument(
        '--rate', type=float, default=2.0,
        help='采样频率（Hz），默认 2'
    )
    parser.add_argument(
        '--output', type=str,
        default=os.path.expanduser('~/rk_inspection_ws/evidence/go2_state_dump'),
        help='输出文件前缀'
    )
    args = parser.parse_args()

    print('=' * 60)
    print('Go2 Read-Only State Logger')
    print('READ-ONLY MODE: NO MOTION COMMANDS WILL BE SENT')
    print(f'Duration: {args.duration}s, Rate: {args.rate}Hz')
    print(f'Output prefix: {args.output}')
    print('=' * 60)

    # 确保输出目录存在
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    rclpy.init(args=None)

    node = Go2ReadonlyStateLogger(
        duration_sec=args.duration,
        rate_hz=args.rate,
        output_prefix=args.output,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
