"""节点接线约束的静态测试，保证感知模块没有控制权限。"""

import os


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
    assert "encoding='bgr8'" in source
    assert "encoding='mono8'" in source
    assert 'expected_color changed' in source


def test_node_assigns_shape_fields_without_gating_grasp_ready():
    source = _node_source()
    assert 'result.shape = candidate.shape' in source
    assert 'result.shape_confidence = float(candidate.shape_confidence)' in source
    assert "'shape={0} conf={1:.2f} vertices={2} rot_aspect={3:.2f}'" in source
    ready_statement = source[source.index('grasp_ready = bool('):source.index('if stale:')]
    assert 'candidate.shape' not in ready_statement
