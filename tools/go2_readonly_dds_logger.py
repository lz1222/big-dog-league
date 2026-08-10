#!/usr/bin/env python3
"""
Go2 只读状态数据采集工具 (DDS 直读版)

READ-ONLY MODE: NO MOTION COMMANDS WILL BE SENT

通过 Unitree SDK2 Python 直接订阅 DDS rt/lowstate，
不发布任何运动控制话题。用于诊断左后腿异常发热和停车左偏问题。

用法:
  python3 go2_readonly_dds_logger.py --duration 30 --rate 5
  python3 go2_readonly_dds_logger.py --duration 60 --rate 2 --output /tmp/diag
"""

import argparse
import csv
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# ── 电机索引常量（来源：Unitree SDK motor_crc.h）──
MOTOR_MAP = [
    ('FR_hip', 0), ('FR_thigh', 1), ('FR_calf', 2),
    ('FL_hip', 3), ('FL_thigh', 4), ('FL_calf', 5),
    ('RR_hip', 6), ('RR_thigh', 7), ('RR_calf', 8),
    ('RL_hip', 9), ('RL_thigh', 10), ('RL_calf', 11),
]

REAR_PAIRS = [
    ('RR_hip', 'RL_hip'),
    ('RR_thigh', 'RL_thigh'),
    ('RR_calf', 'RL_calf'),
]

FOOT_NAMES = ['FR', 'FL', 'RR', 'RL']


def make_fieldnames():
    fields = ['timestamp', 'elapsed_sec']
    for leg in ['FR', 'FL', 'RR', 'RL']:
        for joint in ['hip', 'thigh', 'calf']:
            for attr in ['temp', 'tau_est', 'q', 'dq']:
                fields.append(f'{leg}_{joint}_{attr}')
    for foot in FOOT_NAMES:
        fields.append(f'{foot}_foot_force')
        fields.append(f'{foot}_foot_force_est')
    fields += ['imu_roll', 'imu_pitch', 'imu_yaw']
    return fields


def extract_row(lowstate, elapsed_sec):
    row = {
        'timestamp': datetime.now().isoformat(),
        'elapsed_sec': round(elapsed_sec, 3),
    }
    for name, idx in MOTOR_MAP:
        motor = lowstate.motor_state[idx]
        row[f'{name}_temp'] = motor.temperature
        row[f'{name}_tau_est'] = motor.tau_est
        row[f'{name}_q'] = motor.q
        row[f'{name}_dq'] = motor.dq
    for i, foot in enumerate(FOOT_NAMES):
        row[f'{foot}_foot_force'] = lowstate.foot_force[i]
        row[f'{foot}_foot_force_est'] = lowstate.foot_force_est[i]
    row['imu_roll'] = lowstate.imu_state.rpy[0]
    row['imu_pitch'] = lowstate.imu_state.rpy[1]
    row['imu_yaw'] = lowstate.imu_state.rpy[2]
    return row


class Go2DdsLogger:
    def __init__(self, duration_sec, rate_hz, output_prefix, net_iface):
        self.duration = duration_sec
        self.period = 1.0 / rate_hz
        self.output_prefix = output_prefix
        self.rows = []
        self.latest = None
        self.latest_lock = threading.Lock()
        self.frame_count = 0
        self.start_time = None
        self.running = True
        self.last_record = 0.0

        self._print_banner()

        ChannelFactoryInitialize(0, net_iface)
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self._on_lowstate, 10)
        print(f'[INFO] Subscribed to rt/lowstate on interface {net_iface}')
        print(f'[INFO] Duration={duration_sec}s, Rate={rate_hz}Hz')
        print(f'[INFO] Waiting for DDS data...\n')

    def _print_banner(self):
        print('=' * 60)
        print('Go2 Read-Only DDS State Logger')
        print('READ-ONLY MODE: NO MOTION COMMANDS WILL BE SENT')
        print('=' * 60)

    def _on_lowstate(self, msg):
        self.frame_count += 1
        with self.latest_lock:
            self.latest = msg

    def run(self):
        self.start_time = time.monotonic()
        self.last_record = self.start_time

        try:
            while self.running and (
                time.monotonic() - self.start_time < self.duration
            ):
                now = time.monotonic()
                if now - self.last_record >= self.period:
                    self._record(now)
                    self.last_record = now
                time.sleep(0.01)
        except KeyboardInterrupt:
            print('\n[WARN] Interrupted by user')
        finally:
            self._finish()

    def _record(self, now):
        with self.latest_lock:
            msg = self.latest

        if msg is None:
            elapsed = now - self.start_time
            if elapsed > 3.0:
                print(f'[WARN] T+{elapsed:.1f}s: No DDS data received '
                      f'(frames={self.frame_count})')
            return

        elapsed = now - self.start_time
        row = extract_row(msg, elapsed)
        self.rows.append(row)
        self._print_live(row, elapsed)

    def _print_live(self, row, elapsed):
        parts = []
        for rr_name, rl_name in REAR_PAIRS:
            rl_t = row.get(f'{rl_name}_temp', '?')
            rr_t = row.get(f'{rr_name}_temp', '?')
            rl_tau = row.get(f'{rl_name}_tau_est', '?')
            rr_tau = row.get(f'{rr_name}_tau_est', '?')
            joint_short = rr_name.split('_')[1]
            if isinstance(rl_t, (int, float)) and isinstance(rr_t, (int, float)):
                t_diff = rl_t - rr_t
                parts.append(f'{joint_short} T:RL={rl_t}/RR={rr_t} Δ={t_diff:+d}')
            if isinstance(rl_tau, (int, float)) and isinstance(rr_tau, (int, float)):
                tau_diff = rl_tau - rr_tau
                parts.append(f'τ:RL={rl_tau:.2f}/RR={rr_tau:.2f} Δ={tau_diff:+.2f}')
        line = ' | '.join(parts)
        print(f'T+{elapsed:5.1f}s | {line}')

    def _finish(self):
        self.running = False
        n = len(self.rows)
        print(f'\n[INFO] Done. Frames={self.frame_count}, Records={n}')

        if n == 0:
            print('[ERROR] No data collected.')
            return

        self._save()
        self._analyze()

    def _save(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = f'{self.output_prefix}_{ts}.csv'
        json_path = f'{self.output_prefix}_{ts}.json'
        out_dir = os.path.dirname(csv_path)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)

        fields = make_fieldnames()
        with open(csv_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            w.writerows(self.rows)
        with open(json_path, 'w') as f:
            json.dump(self.rows, f, indent=2, default=str)

        print(f'OUTPUT_CSV={csv_path}')
        print(f'OUTPUT_JSON={json_path}')

    def _analyze(self):
        print('\n========== QUICK ANALYSIS ==========')

        # ── 温度 ──
        print('\n--- 温度对比 (°C) ---')
        for rr_name, rl_name in REAR_PAIRS:
            rr = [r[f'{rr_name}_temp'] for r in self.rows if r.get(f'{rr_name}_temp') is not None]
            rl = [r[f'{rl_name}_temp'] for r in self.rows if r.get(f'{rl_name}_temp') is not None]
            if rr and rl:
                print(f'  {rr_name}: avg={sum(rr)/len(rr):.1f} max={max(rr)}')
                print(f'  {rl_name}: avg={sum(rl)/len(rl):.1f} max={max(rl)}')
                print(f'  温差 Δavg={sum(rl)/len(rl)-sum(rr)/len(rr):.1f}')
                duration = self.rows[-1]['elapsed_sec'] - self.rows[0]['elapsed_sec']
                if len(rl) >= 2 and duration > 0:
                    rate_rl = (rl[-1] - rl[0]) / (duration / 60.0)
                    rate_rr = (rr[-1] - rr[0]) / (duration / 60.0)
                    print(f'  温升: RL={rate_rl:.1f}°C/min, RR={rate_rr:.1f}°C/min')

        # ── 力矩 ──
        print('\n--- 估计力矩 (tau_est, N·m) ---')
        for rr_name, rl_name in REAR_PAIRS:
            rr_t = [r[f'{rr_name}_tau_est'] for r in self.rows if r.get(f'{rr_name}_tau_est') is not None]
            rl_t = [r[f'{rl_name}_tau_est'] for r in self.rows if r.get(f'{rl_name}_tau_est') is not None]
            if rr_t and rl_t:
                avg_rr = sum(rr_t)/len(rr_t)
                avg_rl = sum(rl_t)/len(rl_t)
                std_rr = math.sqrt(sum((x-avg_rr)**2 for x in rr_t)/len(rr_t)) if len(rr_t)>1 else 0
                std_rl = math.sqrt(sum((x-avg_rl)**2 for x in rl_t)/len(rl_t)) if len(rl_t)>1 else 0
                print(f'  {rr_name}: avg={avg_rr:.3f} std={std_rr:.3f}')
                print(f'  {rl_name}: avg={avg_rl:.3f} std={std_rl:.3f}')
                print(f'  力矩差 Δavg={avg_rl-avg_rr:.3f}')

        # ── 足端力 ──
        print('\n--- 足端力分布 ---')
        forces = {}
        for foot in FOOT_NAMES:
            vals = [r.get(f'{foot}_foot_force') for r in self.rows if r.get(f'{foot}_foot_force') is not None]
            forces[foot] = sum(vals)/len(vals) if vals else 0
        total = sum(forces.values())
        if total > 0:
            for foot in FOOT_NAMES:
                print(f'  {foot}: {forces[foot]:.1f} ({forces[foot]/total*100:.1f}%)')
            print(f'  左右后腿力差 RL-RR = {forces["RL"]-forces["RR"]:.1f}')

        # ── 姿态 ──
        print('\n--- 机身姿态 ---')
        for axis, idx in [('roll', 0), ('pitch', 1), ('yaw', 2)]:
            vals = [r[f'imu_{axis}'] for r in self.rows if r.get(f'imu_{axis}') is not None]
            if vals:
                avg = sum(vals)/len(vals)
                print(f'  {axis}: avg={avg:.4f} rad ({math.degrees(avg):.2f}°)')

        print('\n=====================================\n')


def main():
    parser = argparse.ArgumentParser(description='Go2 Read-Only DDS State Logger')
    parser.add_argument('--duration', type=float, default=30.0,
                        help='采集时长（秒），默认 30')
    parser.add_argument('--rate', type=float, default=5.0,
                        help='采样频率（Hz），默认 5')
    parser.add_argument('--output', type=str,
                        default=os.path.expanduser('~/rk_inspection_ws/evidence/go2_dds_dump'),
                        help='输出文件前缀')
    parser.add_argument('--interface', type=str, default='eth0',
                        help='DDS 网络接口，默认 eth0')
    args = parser.parse_args()

    logger = Go2DdsLogger(
        duration_sec=args.duration,
        rate_hz=args.rate,
        output_prefix=args.output,
        net_iface=args.interface,
    )
    logger.run()


if __name__ == '__main__':
    main()
