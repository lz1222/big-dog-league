"""节点接线约束的静态测试，保证感知模块没有控制权限。"""

import os
import sys

import numpy as np


def _debug_image_message_helper():
    """按源码测试路径延迟导入节点辅助函数，避免改变正式包导入顺序。"""
    source_root = os.path.join(os.path.dirname(__file__), '..')
    if source_root not in sys.path:
        sys.path.insert(0, source_root)
    from rk_perception.color_object_detector_node import _debug_image_message
    return _debug_image_message


def _image_decoder_helper():
    """延迟导入，复用正式节点的 D435i 原始 Image 解码路径。"""
    source_root = os.path.join(os.path.dirname(__file__), '..')
    if source_root not in sys.path:
        sys.path.insert(0, source_root)
    from rk_perception.color_object_detector_node import _image_message_to_numpy
    return _image_message_to_numpy


def _node_source():
    path = os.path.join(os.path.dirname(__file__), '..', 'rk_perception',
                        'color_object_detector_node.py')
    with open(path, 'r') as stream:
        return stream.read()


def test_node_uses_ats_sensor_qos_and_image_timestamp():
    source = _node_source()
    assert 'ApproximateTimeSynchronizer' in source
    assert 'qos_profile_sensor_data' in source
    assert 'result.header = image_message.header' in source


def test_node_has_no_motion_or_arm_publishers():
    source = _node_source()
    assert 'cmd_vel' not in source
    assert 'unitree' not in source.lower()
    assert 'DDS' not in source


def test_node_publishes_required_debug_encodings():
    source = _node_source()
    assert "_debug_image_message(overlay, 'bgr8')" in source
    assert "_debug_image_message(mask, 'mono8')" in source
    assert 'expected_color changed' in source


def test_debug_image_message_supports_standard_bgr8_with_opencv5_cvtype():
    """调试图直接封装 bgr8，避免依赖 cv_bridge 的 OpenCV 类型映射。"""
    image = np.zeros((2, 3, 3), dtype=np.uint8)
    image[1, 2] = (10, 20, 30)
    message = _debug_image_message_helper()(image, 'bgr8')

    assert message.encoding == 'bgr8'
    assert (message.height, message.width, message.step) == (2, 3, 9)
    assert bytes(message.data) == image.tobytes()


def test_d435i_image_decoder_converts_rgb8_and_preserves_depth_values():
    """验证实际 D435i 常用 RGB8/16UC1 可转成 OpenCV 输入数组。"""
    from sensor_msgs.msg import Image

    decode = _image_decoder_helper()
    color = Image()
    color.height, color.width, color.step = 1, 2, 6
    color.encoding = 'rgb8'
    color.data.frombytes(bytes([255, 0, 0, 0, 255, 0]))
    bgr = decode(color, 'bgr8')
    assert bgr.tolist() == [[[0, 0, 255], [0, 255, 0]]]

    depth = Image()
    depth.height, depth.width, depth.step = 1, 2, 4
    depth.encoding = '16UC1'
    depth.data.frombytes(np.asarray([500, 850], dtype='<u2').tobytes())
    decoded_depth = decode(depth)
    assert decoded_depth.tolist() == [[500, 850]]


def test_node_assigns_shape_fields_without_gating_grasp_ready():
    source = _node_source()
    assert 'result.shape = candidate.shape' in source
    assert 'result.shape_confidence = float(candidate.shape_confidence)' in source
    assert "'shape={0} conf={1:.2f} vertices={2} rot_aspect={3:.2f}'" in source
    ready_statement = source[source.index('grasp_ready = bool('):source.index('if stale:')]
    assert 'candidate.shape' not in ready_statement
    assert 'object_list_publisher' in source
    assert '_publish_object_list' in source
