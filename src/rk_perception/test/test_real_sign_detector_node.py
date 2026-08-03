import cv2
import numpy as np

from rk_perception.real_sign_detector_node import (
    PlaceMarkerConfirmation,
    PlaceMarkerFrameResult,
    SignCandidate,
    _rotate_mask,
    _template_score,
    confirm_place_marker_history,
    detect_color_signs,
    detect_place_marker_candidates,
    load_place_marker_templates,
    load_warning_templates,
    merge_candidates,
    normalize_label,
    parse_color_rules,
)


def _make_place_marker(template, angle=0):
    """用同一归一化模板构造红环实体样本，避免测试假设编号必须是阿拉伯数字。"""
    image = np.full((360, 480, 3), 255, dtype=np.uint8)
    center = (240, 180)
    cv2.circle(image, center, 92, (0, 0, 255), 14)
    # 红环外缘约为 198 像素，按生产归一化方向反投影内部黑色图案。
    diameter = 198
    x0 = center[0] - diameter // 2
    y0 = center[1] - diameter // 2
    mask = cv2.resize(template, (diameter, diameter),
                      interpolation=cv2.INTER_NEAREST)
    region = image[y0:y0 + diameter, x0:x0 + diameter]
    region[mask > 127] = (0, 0, 0)
    if angle:
        matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        image = cv2.warpAffine(image, matrix, (480, 360),
                               borderValue=(255, 255, 255))
    return image


def _place_result(value, confidence=0.90):
    candidate = None
    if value is not None:
        candidate = SignCandidate(
            sign_type='place_marker', sign_value=value,
            confidence=confidence, source='test')
    return PlaceMarkerFrameResult(candidate=candidate)


def _assert_place_unknown(image, templates=None, **kwargs):
    result = detect_place_marker_candidates(
        image, templates or load_place_marker_templates(), **kwargs)
    assert result.candidate is None


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


def test_place_marker_templates_are_auditable_and_distinct():
    templates = load_place_marker_templates()
    assert set(templates) == {'place_1', 'place_2'}
    assert templates['place_1'].shape == (128, 128)
    assert templates['place_2'].shape == (128, 128)
    assert _template_score(templates['place_1'], templates['place_2']) < 0.75


def test_place_1_standard_template_is_correct_and_not_place_2():
    templates = load_place_marker_templates()
    result = detect_place_marker_candidates(
        _make_place_marker(templates['place_1']), templates)
    assert result.candidate is not None
    assert result.candidate.sign_value == 'place_1'
    assert result.place_1_score > result.place_2_score


def test_place_2_standard_template_is_correct_and_not_place_1():
    templates = load_place_marker_templates()
    result = detect_place_marker_candidates(
        _make_place_marker(templates['place_2']), templates)
    assert result.candidate is not None
    assert result.candidate.sign_value == 'place_2'
    assert result.place_2_score > result.place_1_score


def test_place_markers_allow_only_small_angle_rotation_search():
    templates = load_place_marker_templates()
    for template, expected in ((templates['place_1'], 'place_1'),
                               (templates['place_2'], 'place_2')):
        result = detect_place_marker_candidates(
            _make_place_marker(template, angle=15), templates)
        assert result.candidate is not None
        assert result.candidate.sign_value == expected


def test_plain_red_ring_and_red_filled_circle_are_not_place_markers():
    image = np.full((360, 480, 3), 255, dtype=np.uint8)
    cv2.circle(image, (240, 180), 92, (0, 0, 255), 14)
    _assert_place_unknown(image)
    filled = np.full((360, 480, 3), 255, dtype=np.uint8)
    cv2.circle(filled, (240, 180), 92, (0, 0, 255), -1)
    _assert_place_unknown(filled)


def test_red_rectangle_and_nonmatching_red_circle_are_not_place_markers():
    rectangle = np.full((360, 480, 3), 255, dtype=np.uint8)
    cv2.rectangle(rectangle, (120, 110), (360, 250), (0, 0, 255), -1)
    _assert_place_unknown(rectangle)
    circle = np.full((360, 480, 3), 255, dtype=np.uint8)
    cv2.circle(circle, (240, 180), 92, (0, 0, 255), 14)
    cv2.circle(circle, (240, 180), 25, (0, 0, 0), -1)
    _assert_place_unknown(circle)


def test_warning_signs_are_not_place_markers():
    # 黄色三角警示牌（包含三种语义）没有红环，必须被平台路径忽略。
    image = np.full((360, 480, 3), 255, dtype=np.uint8)
    triangle = np.array([[240, 75], [110, 290], [370, 290]], dtype=np.int32)
    cv2.fillConvexPoly(image, triangle, (0, 220, 255))
    for warning_name in ('warning', 'radiation', 'electric_shock',
                         'strong_oxidizer'):
        _assert_place_unknown(image), warning_name


def test_place_marker_rejects_insufficient_score_and_margin():
    templates = load_place_marker_templates()
    _assert_place_unknown(
        _make_place_marker(templates['place_1']), templates, min_score=0.99)
    same_templates = {'place_1': templates['place_1'],
                      'place_2': templates['place_1']}
    _assert_place_unknown(
        _make_place_marker(templates['place_1']), same_templates,
        min_margin=0.12)


def test_place_marker_confirmation_requires_five_of_seven_for_each_class():
    for value in ('place_1', 'place_2'):
        history = [_place_result(value) for _ in range(5)]
        history.extend([_place_result(None), _place_result(None)])
        status = confirm_place_marker_history(history)
        assert status['confirmed'] is True
        assert status['confirmed_value'] == value
        assert status['confirm_count'] == 5


def test_place_marker_confirmation_rejects_conflict_and_single_frame():
    confirmer = PlaceMarkerConfirmation(7, 5, 0.75)
    status = confirmer.update(_place_result('place_1'))
    assert status['confirmed'] is False
    assert status['confirmed_value'] == 'unknown'
    history = [_place_result('place_1') for _ in range(5)]
    history.append(_place_result('place_2'))
    status = confirm_place_marker_history(history)
    assert status['confirmed'] is False
    assert status['confirmed_value'] == 'unknown'
