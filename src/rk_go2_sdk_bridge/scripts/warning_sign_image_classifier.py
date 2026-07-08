#!/usr/bin/env python3

import argparse
import json
import os
import sys

import cv2


ACTION_BY_WARNING_SIGN = {
    'electric_shock': 'stretch',
    'strong_oxidizer': 'hello',
    'radiation': 'blink_front_light_3',
}


def add_workspace_python_paths():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
    rk_perception_src = os.path.join(workspace, 'src', 'rk_perception')
    if os.path.isdir(rk_perception_src):
        sys.path.insert(0, rk_perception_src)


def parse_args():
    parser = argparse.ArgumentParser(
        description='Classify a competition warning sign image without ROS2.'
    )
    parser.add_argument('image_path', help='JPEG/PNG image captured from Go2.')
    parser.add_argument(
        '--min-confidence',
        type=float,
        default=0.50,
        help='Minimum detection confidence to accept.'
    )
    parser.add_argument(
        '--template-min-score',
        type=float,
        default=0.10,
        help='Minimum inner-symbol template score.'
    )
    parser.add_argument(
        '--min-area-fraction',
        type=float,
        default=0.010,
        help='Minimum yellow warning-sign area fraction.'
    )
    parser.add_argument(
        '--action-only',
        action='store_true',
        help='Print only the mapped SDK action when detected.'
    )
    return parser.parse_args()


def main():
    args = parse_args()
    add_workspace_python_paths()

    try:
        from rk_perception.real_sign_detector_node import (
            detect_warning_template_signs,
            load_warning_templates,
            merge_candidates,
        )
    except Exception as error:
        print(json.dumps({
            'detected': False,
            'error': f'failed to import detector: {error}',
        }, ensure_ascii=False))
        return 1

    image = cv2.imread(args.image_path, cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        print(json.dumps({
            'detected': False,
            'error': f'failed to read image: {args.image_path}',
        }, ensure_ascii=False))
        return 1

    templates = load_warning_templates()
    candidates = detect_warning_template_signs(
        image,
        templates,
        min_area_fraction=args.min_area_fraction,
        min_score=args.template_min_score
    )
    detections = merge_candidates(candidates, args.min_confidence)

    if not detections:
        result = {
            'detected': False,
            'image_path': args.image_path,
        }
        if args.action_only:
            return 2
        print(json.dumps(result, ensure_ascii=False, separators=(',', ':')))
        return 2

    best = detections[0]
    action = ACTION_BY_WARNING_SIGN.get(best.sign_value, '')
    result = {
        'detected': True,
        'sign_type': best.sign_type,
        'sign_value': best.sign_value,
        'confidence': round(float(best.confidence), 4),
        'source': best.source,
        'action': action,
        'image_path': args.image_path,
    }

    if args.action_only:
        print(action)
    else:
        print(json.dumps(result, ensure_ascii=False, separators=(',', ':')))
    return 0 if action else 3


if __name__ == '__main__':
    sys.exit(main())
