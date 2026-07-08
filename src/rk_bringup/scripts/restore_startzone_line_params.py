#!/usr/bin/env python3

from pathlib import Path


GOOD_VALUES = {
    'enable_debug_image': 'false',
    'robot_center_x_offset_fraction': '0.0',
    'robot_center_x_offset_px': '0.0',
    'min_path_bands': '4',
    'min_valid_bands': '4',
    'require_bottom_band': 'true',
    'max_line_width_fraction': '0.26',
    'max_line_width_top_fraction': '0.12',
    'max_line_width_bottom_fraction': '0.42',
    'max_dark_fraction': '0.22',
    'visible_min_confidence': '0.35',
    'max_track_jump_fraction': '0.18',
    'max_reacquire_jump_fraction': '0.12',
    'bottom_band_preference_weight': '8.0',
    'previous_center_weight': '10.0',
    'bottom_start_max_center_error_fraction': '0.30',
    'max_band_center_jump_fraction': '0.16',
    'max_band_gap': '2',
    'min_driving_speed': '0.27',
    'base_speed': '0.27',
    'mid_speed': '0.27',
    'slow_speed': '0.27',
    'error_slow_threshold': '0.12',
    'error_slowest_threshold': '0.28',
    'kp_lateral': '0.85',
    'kp_heading': '0.35',
    'max_angular_z': '0.28',
    'angular_deadband': '0.08',
    'angular_smoothing_alpha': '0.22',
    'short_lost_timeout': '0.75',
    'search_angular_speed': '0.25',
    'turn_90_angular_speed': '0.35',
    'lost_turn_angular_speed': '0.25',
    'search_line_angular_speed': '0.22',
}


def main():
    config_path = Path(__file__).resolve().parents[1] / 'config' / (
        'line_nav_params.yaml'
    )
    lines = config_path.read_text(encoding='utf-8').splitlines()
    updated = []
    changed = []

    for line in lines:
        stripped = line.lstrip()
        indent = line[:len(line) - len(stripped)]
        key = stripped.split(':', 1)[0] if ':' in stripped else ''
        if key in GOOD_VALUES:
            new_line = f'{indent}{key}: {GOOD_VALUES[key]}'
            if new_line != line:
                old_value = stripped.split(':', 1)[1].strip()
                changed.append(f'{key}: {old_value} -> {GOOD_VALUES[key]}')
            updated.append(new_line)
        else:
            updated.append(line)

    config_path.write_text('\n'.join(updated) + '\n', encoding='utf-8')
    if changed:
        print('Restored start-zone line parameters:')
        for item in changed:
            print(f'  {item}')
    else:
        print('Start-zone line parameters already match the known-good values.')


if __name__ == '__main__':
    main()
