import base64

import cv2
import numpy as np

from rk_perception.real_sign_detector_node import (
    DEFAULT_WARNING_TEMPLATE_IMAGES,
    detect_color_signs,
    detect_warning_template_signs,
    load_warning_templates,
    merge_candidates,
    normalize_label,
    parse_color_rules,
)


def test_normalize_label():
    assert normalize_label('Electric Shock') == 'electric_shock'
    assert normalize_label('place-1') == 'place_1'


def test_detect_color_signs_red_warning():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (40, 30), (120, 90), (0, 0, 255), -1)

    rules = parse_color_rules('')
    candidates = detect_color_signs(image, rules)
    detections = merge_candidates(candidates, 0.55)

    assert detections
    assert detections[0].sign_type == 'warning'
    assert detections[0].sign_value == 'electric_shock'
    assert detections[0].confidence >= 0.55


def test_detect_warning_template_signs():
    templates = load_warning_templates()
    assert set(templates) == {
        'electric_shock',
        'strong_oxidizer',
        'radiation',
    }

    for sign_value, encoded in DEFAULT_WARNING_TEMPLATE_IMAGES.items():
        image_bytes = base64.b64decode(encoded)
        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        candidates = detect_warning_template_signs(
            image,
            templates,
            min_area_fraction=0.001,
            min_score=0.30
        )
        detections = merge_candidates(candidates, 0.55)

        assert detections
        assert detections[0].sign_type == 'warning'
        assert detections[0].sign_value == sign_value
