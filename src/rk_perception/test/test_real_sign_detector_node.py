import cv2
import numpy as np
import pytest

from rk_perception.real_sign_detector_node import (
    _rotate_mask,
    _template_score,
    detect_color_signs,
    detect_qr_signs,
    detect_warning_template_signs,
    image_transport_qos,
    image_message_to_owned_bgr8,
    load_warning_templates,
    merge_candidates,
    normalize_label,
    parse_color_rules,
    parse_qr_value_map,
    prepare_qr_image,
)


class _FakeImage:
    """脱离 ROS runtime 构造 Image 边界条件，专测输入内存所有权。"""

    def __init__(self, width, height, step, data, encoding='bgr8'):
        self.width = width
        self.height = height
        self.step = step
        self.data = data
        self.encoding = encoding


def _padded_image(width=4, height=3, padding=0):
    step = width * 3 + padding
    data = bytearray(height * step)
    for y in range(height):
        for x in range(width * 3):
            data[y * step + x] = (x + y * 31) % 256
    return _FakeImage(width, height, step, data), data


def test_manual_bgr8_conversion_owns_contiguous_memory():
    msg, raw = _padded_image()
    image = image_message_to_owned_bgr8(msg)

    assert image.shape == (3, 4, 3)
    assert image.flags.c_contiguous
    assert image.flags.owndata
    before = image.copy()
    raw[0] = (raw[0] + 1) % 256
    assert np.array_equal(image, before), 'output must not alias message data'


def test_manual_bgr8_conversion_honours_row_padding():
    msg, _raw = _padded_image(width=3, height=2, padding=5)
    image = image_message_to_owned_bgr8(msg)

    assert image[0, 0].tolist() == [0, 1, 2]
    assert image[1, 0].tolist() == [31, 32, 33]


@pytest.mark.parametrize(
    'msg',
    [
        _FakeImage(0, 1, 3, bytearray(3)),
        _FakeImage(1, 0, 3, bytearray(3)),
        _FakeImage(2, 1, 5, bytearray(6)),
        _FakeImage(2, 1, 6, bytearray(5)),
        _FakeImage(1, 1, 3, bytearray(3), encoding='rgb8'),
    ],
)
def test_manual_bgr8_conversion_rejects_invalid_frames(msg):
    with pytest.raises(ValueError):
        image_message_to_owned_bgr8(msg)


def test_1920x1080_manual_conversion_1000_repeats():
    """大图转换压力回归：只验证安全复制，不把 DDS 缓冲区传给 OpenCV。"""
    width, height = 1920, 1080
    msg = _FakeImage(width, height, width * 3, bytearray(width * height * 3))
    for _ in range(1000):
        image = image_message_to_owned_bgr8(msg)
        assert image.shape == (height, width, 3)
        assert image.flags.c_contiguous and image.flags.owndata


def test_qr_input_is_downscaled_and_isolated():
    class _ProbeDetector:
        def __init__(self):
            self.shapes = []

        def detectAndDecodeMulti(self, image):
            self.shapes.append(image.shape)
            return False, (), None, None

        def detectAndDecode(self, image):
            self.shapes.append(image.shape)
            return '', None, None

    source = np.zeros((1080, 1920, 3), dtype=np.uint8)
    prepared = prepare_qr_image(source, 960)
    assert prepared.shape == (540, 960, 3)
    assert prepared.flags.c_contiguous and prepared.flags.owndata
    source[:, :] = 255
    assert not np.any(prepared), 'QR input must not alias the source image'

    detector = _ProbeDetector()
    assert detect_qr_signs(source, detector, {}, 960) == []
    assert detector.shapes == [(540, 960, 3), (540, 960, 3)]


def test_normalize_label():
    assert normalize_label('Electric Shock') == 'electric_shock'
    assert normalize_label('place-1') == 'place_1'


def test_detect_color_signs_red_warning():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (40, 30), (120, 90), (0, 0, 255), -1)

    rules = parse_color_rules('')
    candidates = detect_color_signs(image, rules)
    detections = merge_candidates(candidates, 0.55)

    assert candidates, '颜色候选仍可为后续定位提供辅助信息'
    assert detections == [], '普通红色矩形不得独立确认 electric_shock'


@pytest.mark.parametrize(
    ('bgr', 'forbidden_value'),
    [
        ((0, 0, 255), 'electric_shock'),
        ((255, 0, 0), 'radiation'),
    ],
)
def test_daily_red_or_blue_objects_do_not_produce_warning(bgr,
                                                          forbidden_value):
    """日常红蓝物体只能产生颜色候选，不能作为最终 warning。"""
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (30, 25), (130, 95), bgr, -1)
    candidates = detect_color_signs(image, parse_color_rules(''))
    detections = merge_candidates(candidates, 0.55)
    assert all(item.sign_value != forbidden_value for item in detections)
    assert not any(item.sign_type == 'warning' for item in detections)


def test_place_marker_colour_rules_are_unchanged():
    rules = parse_color_rules('')
    samples = [
        ((0, 255, 0), 'place_1'),
        ((255, 0, 255), 'place_2'),
    ]
    for bgr, expected in samples:
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        cv2.rectangle(image, (30, 25), (130, 95), bgr, -1)
        detections = merge_candidates(detect_color_signs(image, rules), 0.55)
        assert any(item.sign_value == expected for item in detections)


def test_detector_image_qos_is_reliable_keep_last_one():
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

    qos = image_transport_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_all_warning_templates_still_produce_their_own_classification():
    """模板仍是三类 warning 的唯一非 QR 最终确认来源。"""
    templates = load_warning_templates()
    for expected, template in templates.items():
        # 在黄底牌面放置模板的黑色符号，模拟实体牌的基本颜色结构。
        symbol = cv2.resize(template, (160, 160),
                            interpolation=cv2.INTER_NEAREST)
        image = np.where(
            symbol[..., None] > 0,
            np.array([0, 0, 0], dtype=np.uint8),
            np.array([0, 255, 255], dtype=np.uint8),
        ).astype(np.uint8)
        detections = detect_warning_template_signs(image, templates)
        assert detections
        assert detections[0].sign_type == 'warning'
        assert detections[0].sign_value == expected
        assert detections[0].confidence >= 0.70


def test_qr_warning_mapping_remains_a_final_warning_source():
    class _WarningQrDetector:
        def detectAndDecodeMulti(self, _image):
            return True, ('radiation',), np.array(
                [[[10.0, 10.0], [30.0, 10.0],
                 [30.0, 30.0], [10.0, 30.0]]]), None

        def detectAndDecode(self, _image):
            return '', None, None

    image = np.zeros((120, 160, 3), dtype=np.uint8)
    candidates = detect_qr_signs(
        image, _WarningQrDetector(), parse_qr_value_map(''), 960)
    detections = merge_candidates(candidates, 0.55)
    assert len(detections) == 1
    assert detections[0].sign_type == 'warning'
    assert detections[0].sign_value == 'radiation'


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
