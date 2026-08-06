#!/usr/bin/env python3
"""T6: 停车偏差分析工具。

读取 collect_stop_bias.py 采集的 JSON 数据，计算:
  - 中位数偏差 (stop_bias_estimate)
  - 方向一致性
  - 标准差
  - 是否启用前馈补偿

用法:
  python3 tools/analyze_stop_bias.py evidence/stop_bias/*.json
  python3 tools/analyze_stop_bias.py --all  # 分析所有采集数据
"""

import argparse
import json
import math
import sys
from pathlib import Path


def analyze_file(path: str) -> dict:
    """Analyze one stop bias data file."""
    with open(path) as f:
        data = json.load(f)

    records = data.get('records', [])
    if not records:
        return {'path': path, 'error': 'no records'}

    shifts = [r['stop_yaw_shift_rad'] for r in records]
    shifts_deg = [math.degrees(s) for s in shifts]
    n = len(shifts)

    sorted_shifts = sorted(shifts)
    mid = n // 2
    median = sorted_shifts[mid] if n % 2 else (sorted_shifts[mid-1] + sorted_shifts[mid]) / 2

    mean = sum(shifts) / n
    variance = sum((s - mean)**2 for s in shifts) / n
    std_dev = math.sqrt(variance)

    dominant = 1 if median >= 0 else -1
    consistent = sum(1 for s in shifts if (s >= 0) == (dominant >= 0))
    consistency = consistent / n

    return {
        'path': path,
        'label': data.get('label', 'unknown'),
        'sample_count': n,
        'median_shift_deg': round(math.degrees(median), 2),
        'mean_shift_deg': round(math.degrees(mean), 2),
        'std_dev_deg': round(math.degrees(std_dev), 2),
        'direction_consistency': round(consistency, 2),
        'dominant_direction': 'LEFT' if dominant > 0 else 'RIGHT',
        'enabled': (n >= 10 and math.degrees(std_dev) < 3.0 and consistency >= 0.7),
        'compensation_ratio_deg': round(math.degrees(median) * 0.30, 2),
    }


def main():
    parser = argparse.ArgumentParser(description='T6 stop bias analyzer')
    parser.add_argument('files', nargs='*', help='JSON files to analyze')
    parser.add_argument('--all', '-a', action='store_true',
                        help='Analyze all files in evidence/stop_bias/')
    args = parser.parse_args()

    paths = list(args.files)

    if args.all:
        bias_dir = Path.home() / 'rk_inspection_ws' / 'evidence' / 'stop_bias'
        if bias_dir.exists():
            paths.extend(sorted(bias_dir.glob('*.json')))

    if not paths:
        print('No files specified. Use --all or provide file paths.')
        print('Example: python3 tools/analyze_stop_bias.py evidence/stop_bias/*.json')
        return 1

    results = [analyze_file(str(p)) for p in paths]

    print(f'{"="*65}')
    print(f'T6 停车偏差分析 ({len(results)} sessions)')
    print(f'{"="*65}')
    print(f'{"标签":<20s} {"样本":>4s} {"中值偏差":>8s} {"σ":>6s} {"一致性":>6s} {"补偿":>8s} {"启用":>4s}')
    print(f'{"-"*65}')

    for r in results:
        if 'error' in r:
            print(f'{r["path"]:<20s} ERROR: {r["error"]}')
            continue
        comp = f'{r["compensation_ratio_deg"]:+.2f}deg'
        en = 'YES' if r['enabled'] else 'no'
        print(f'{r["label"]:<20s} {r["sample_count"]:4d} '
              f'{r["median_shift_deg"]:+7.2f}deg '
              f'{r["std_dev_deg"]:5.2f} '
              f'{r["direction_consistency"]:5.0%} '
              f'{comp:>8s} '
              f'{en:>4s}')

    print(f'{"="*65}')

    # Overall recommendation
    enabled = [r for r in results if not 'error' in r and r['enabled']]
    if enabled:
        print(f'\n{len(enabled)} session(s) meet criteria for feedforward.')
        for r in enabled:
            print(f'  {r["label"]}: compensate {r["compensation_ratio_deg"]} '
                  f'({r["dominant_direction"]} side, {r["sample_count"]} samples)')
    else:
        print(f'\nNo sessions meet criteria (need >=10 samples, σ<3.0deg, consistency>=70%).')
        insufficient = [r for r in results if not 'error' in r and not r['enabled']]
        for r in insufficient:
            reasons = []
            if r['sample_count'] < 10:
                reasons.append(f'need {10-r["sample_count"]} more samples')
            if r['std_dev_deg'] >= 3.0:
                reasons.append(f'σ={r["std_dev_deg"]:.1f}deg > 3.0')
            if r['direction_consistency'] < 0.7:
                reasons.append(f'consistency={r["direction_consistency"]:.0%} < 70%')
            print(f'  {r["label"]}: {", ".join(reasons)}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
