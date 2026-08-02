import cv2
import numpy as np

from rk_perception.real_sign_detector_node import (
    _rotate_mask,
    _template_score,
    detect_color_signs,
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
    }, 'Three warning templates must be present'

    # Each template must match itself with high confidence.
    for sign_value, template in templates.items():
        score = _template_score(template, template)
        assert score > 0.90, (
            '%s self-match too low: %.4f' % (sign_value, score))
        # Rotation should still give high self-match.
        for angle in (-15, 0, 15):
            rot = _rotate_mask(template, angle)
            if rot is not None:
                rot_score = _template_score(template, rot)
                assert rot_score > 0.85, (
                    '%s @ %+d self-match too low: %.4f'
                    % (sign_value, angle, rot_score))

    # Cross-match: no two templates should score too close.
    names = sorted(templates)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            cross = _template_score(templates[names[i]],
                                    templates[names[j]])
            assert cross < 0.70, (
                '%s vs %s cross-match too high: %.4f'
                % (names[i], names[j], cross))

    # Template properties: all must be 48x48 uint8.
    for sign_value, template in templates.items():
        assert template.shape == (48, 48), (
            '%s wrong shape: %s' % (sign_value, template.shape))
        assert template.dtype == np.uint8, (
            '%s wrong dtype: %s' % (sign_value, template.dtype))

    # Missing explicit resource_dir falls back to defaults — OK.
    # Corrupted template must be skipped (fail-closed).
    import os
    import tempfile
    with tempfile.TemporaryDirectory() as tmpdir:
        # Create a corrupt PNG
        bad = os.path.join(tmpdir, 'corrupt.png')
        with open(bad, 'wb') as f:
            f.write(b'not a png file')
        with_tmp = load_warning_templates(resource_dir=tmpdir)
        # The corrupt file must not produce a template.
        assert 'corrupt' not in with_tmp
