from importlib import import_module
import json
from pathlib import Path
import sys
import time
import uuid

import pytest


rclpy = pytest.importorskip('rclpy')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
std_msgs = pytest.importorskip('std_msgs.msg')
std_srvs = pytest.importorskip('std_srvs.srv')

PACKAGE_ROOT = str(Path(__file__).resolve().parents[1])
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

command_mux_node = import_module('rk_safety.command_mux_node')
CommandMuxNode = command_mux_node.CommandMuxNode
ESTOP_STATE_QOS = command_mux_node.ESTOP_STATE_QOS
VelocityCommand = command_mux_node.VelocityCommand
Twist = geometry_msgs.Twist
Bool = std_msgs.Bool
String = std_msgs.String
SetBool = std_srvs.SetBool


def _make_topics():
    prefix = '/rk_safety_estop_state_test_{}'.format(uuid.uuid4().hex)
    return {
        'line': prefix + '/line',
        'mission': prefix + '/mission',
        'locomotion': prefix + '/locomotion',
        'estop': prefix + '/estop',
        'estop_state': prefix + '/estop_state',
        'gait_lock': prefix + '/gait_lock',
        'arm_lock': prefix + '/arm_lock',
        'output': prefix + '/output',
        'status': prefix + '/status',
        'estop_service': prefix + '/estop_service',
    }


def _ros_args(topics):
    arguments = ['--ros-args']
    parameters = {
        'line_cmd_topic': topics['line'],
        'mission_cmd_topic': topics['mission'],
        'locomotion_cmd_topic': topics['locomotion'],
        'estop_topic': topics['estop'],
        'estop_state_topic': topics['estop_state'],
        'gait_lock_topic': topics['gait_lock'],
        'arm_lock_topic': topics['arm_lock'],
        'output_cmd_topic': topics['output'],
        'status_topic': topics['status'],
        'estop_service_name': topics['estop_service'],
    }
    for name, value in parameters.items():
        arguments.extend(['-p', '{}:={}'.format(name, value)])
    arguments.extend(['-p', 'enable_estop_service:=true'])
    return arguments


def _spin_until(executor, predicate, timeout_sec=5.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError('Timed out waiting for command mux test condition')


def _call_estop(executor, client, enabled):
    request = SetBool.Request()
    request.data = enabled
    future = client.call_async(request)
    _spin_until(executor, future.done)
    response = future.result()
    assert response is not None
    assert response.success is True
    return response


def _shutdown_ros(executor, nodes):
    if executor is not None:
        for node in nodes:
            if node is not None:
                executor.remove_node(node)
        executor.shutdown()
    for node in reversed(nodes):
        if node is not None:
            node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_estop_state_initial_topic_service_and_repeated_transitions():
    topics = _make_topics()
    rclpy.init(args=_ros_args(topics))
    mux = None
    probe = None
    executor = None
    try:
        mux = CommandMuxNode()
        mux._timer.cancel()
        probe = rclpy.create_node(
            'estop_state_transition_probe_{}'.format(uuid.uuid4().hex[:12])
        )
        states = []
        state_subscription = probe.create_subscription(
            Bool,
            topics['estop_state'],
            lambda message: states.append(bool(message.data)),
            ESTOP_STATE_QOS,
        )
        estop_publisher = probe.create_publisher(
            Bool, topics['estop'], 10
        )
        estop_client = probe.create_client(
            SetBool, topics['estop_service']
        )
        assert state_subscription is not None

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(probe)
        _spin_until(
            executor,
            lambda: states
            and estop_publisher.get_subscription_count() > 0
            and estop_client.service_is_ready(),
        )
        assert states[-1] is False
        assert mux._core.estop is False

        def publish_estop(enabled):
            states[:] = []
            message = Bool()
            message.data = enabled
            estop_publisher.publish(message)
            _spin_until(executor, lambda: states)
            assert states[-1] is enabled
            assert mux._core.estop is enabled

        publish_estop(True)
        publish_estop(True)
        publish_estop(False)

        now = mux._now_sec()
        assert mux._core.update_mission_command(
            VelocityCommand(linear_x=0.2), now
        )
        assert mux._core.evaluate(now).active_source == 'mission'

        states[:] = []
        response = _call_estop(executor, estop_client, True)
        _spin_until(executor, lambda: states)
        assert states[-1] is True
        assert mux._core.estop is True
        assert response.message == (
            'Emergency stop enabled; command caches cleared'
        )

        states[:] = []
        response = _call_estop(executor, estop_client, True)
        _spin_until(executor, lambda: states)
        assert states[-1] is True
        assert mux._core.estop is True
        assert response.message == (
            'Emergency stop already enabled; state unchanged'
        )

        states[:] = []
        response = _call_estop(executor, estop_client, False)
        _spin_until(executor, lambda: states)
        assert states[-1] is False
        assert mux._core.estop is False
        assert response.message == (
            'Emergency stop cleared; waiting for new command'
        )

        decision = mux._core.evaluate(mux._now_sec())
        assert decision.command == VelocityCommand()
        assert decision.active_source == 'none'
        assert decision.status['mission_fresh'] is False
        assert decision.status['mission_age_sec'] is None
    finally:
        _shutdown_ros(executor, [mux, probe])


def test_estop_state_heartbeat_tracks_core_state():
    topics = _make_topics()
    rclpy.init(args=_ros_args(topics))
    mux = None
    probe = None
    executor = None
    try:
        mux = CommandMuxNode()
        probe = rclpy.create_node(
            'estop_state_heartbeat_probe_{}'.format(uuid.uuid4().hex[:12])
        )
        states = []
        state_subscription = probe.create_subscription(
            Bool,
            topics['estop_state'],
            lambda message: states.append(bool(message.data)),
            ESTOP_STATE_QOS,
        )
        estop_publisher = probe.create_publisher(
            Bool, topics['estop'], 10
        )
        assert state_subscription is not None

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(probe)
        _spin_until(
            executor,
            lambda: states
            and estop_publisher.get_subscription_count() > 0,
        )

        states[:] = []
        _spin_until(executor, lambda: len(states) >= 3)
        assert states[:3] == [False, False, False]
        assert all(state is bool(mux._core.estop) for state in states)

        states[:] = []
        message = Bool()
        message.data = True
        estop_publisher.publish(message)
        _spin_until(executor, lambda: True in states)
        assert mux._core.estop is True

        states[:] = []
        _spin_until(executor, lambda: len(states) >= 3)
        assert states[:3] == [True, True, True]
        assert all(state is bool(mux._core.estop) for state in states)
    finally:
        _shutdown_ros(executor, [mux, probe])


def test_estop_state_transient_local_late_subscribers():
    topics = _make_topics()
    rclpy.init(args=_ros_args(topics))
    mux = None
    probe_a = None
    probe_b = None
    executor = None
    try:
        mux = CommandMuxNode()
        mux._timer.cancel()
        probe_a = rclpy.create_node(
            'estop_state_late_probe_a_{}'.format(uuid.uuid4().hex[:12])
        )
        states_a = []
        subscription_a = probe_a.create_subscription(
            Bool,
            topics['estop_state'],
            lambda message: states_a.append(bool(message.data)),
            ESTOP_STATE_QOS,
        )
        estop_publisher = probe_a.create_publisher(
            Bool, topics['estop'], 10
        )
        assert subscription_a is not None

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(probe_a)
        _spin_until(
            executor,
            lambda: states_a
            and estop_publisher.get_subscription_count() > 0,
        )
        assert states_a[-1] is False

        states_a[:] = []
        message = Bool()
        message.data = True
        estop_publisher.publish(message)
        _spin_until(executor, lambda: states_a == [True])
        assert mux._core.estop is True

        probe_b = rclpy.create_node(
            'estop_state_late_probe_b_{}'.format(uuid.uuid4().hex[:12])
        )
        states_b = []
        subscription_b = probe_b.create_subscription(
            Bool,
            topics['estop_state'],
            lambda message: states_b.append(bool(message.data)),
            ESTOP_STATE_QOS,
        )
        assert subscription_b is not None
        executor.add_node(probe_b)
        _spin_until(executor, lambda: states_b)
        assert states_b[-1] is True

        states_a[:] = []
        states_b[:] = []
        message.data = False
        estop_publisher.publish(message)
        _spin_until(
            executor,
            lambda: states_a
            and states_b
            and states_a[-1] is False
            and states_b[-1] is False,
        )
        assert mux._core.estop is False
    finally:
        _shutdown_ros(executor, [mux, probe_a, probe_b])


class _RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _FailingPublisher:
    def __init__(self):
        self.calls = 0

    def publish(self, message):
        self.calls += 1
        raise RuntimeError('injected typed estop publisher failure')


class _RecordingLogger:
    def __init__(self, raises=False):
        self.messages = []
        self.raises = raises

    def error(self, message):
        self.messages.append(message)
        if self.raises:
            raise RuntimeError('injected logger failure')


def test_estop_state_publish_failure_isolated_and_throttled(monkeypatch):
    topics = _make_topics()
    rclpy.init(args=_ros_args(topics))
    mux = None
    try:
        mux = CommandMuxNode()
        mux._timer.cancel()
        original_command_publisher = mux._command_publisher
        original_status_publisher = mux._status_publisher
        original_estop_state_publisher = mux._estop_state_publisher

        command_publisher = _RecordingPublisher()
        status_publisher = _RecordingPublisher()
        failing_publisher = _FailingPublisher()
        logger = _RecordingLogger()
        current_time = [10.0]

        mux._command_publisher = command_publisher
        mux._status_publisher = status_publisher
        mux._estop_state_publisher = failing_publisher
        monkeypatch.setattr(
            command_mux_node.time, 'monotonic', lambda: current_time[0]
        )
        monkeypatch.setattr(mux, 'get_logger', lambda: logger)

        message = Bool()
        message.data = True
        mux._on_estop(message)
        assert mux._core.estop is True
        assert len(logger.messages) == 1

        current_time[0] = 11.0
        mux._publish_decision()
        assert mux._core.estop is True
        assert len(logger.messages) == 1
        assert len(command_publisher.messages) == 1
        output = command_publisher.messages[-1]
        assert output.linear.x == 0.0
        assert output.linear.y == 0.0
        assert output.angular.z == 0.0
        assert len(status_publisher.messages) == 1
        status = json.loads(status_publisher.messages[-1].data)
        assert status['estop'] is True
        assert status['active_source'] == 'estop'

        request = SetBool.Request()
        request.data = True
        response = mux._on_estop_service(request, SetBool.Response())
        assert response.success is True
        assert response.message == (
            'Emergency stop already enabled; state unchanged'
        )
        assert mux._core.estop is True

        current_time[0] = 15.0
        mux._publish_estop_state()
        assert len(logger.messages) == 2

        successful_publisher = _RecordingPublisher()
        mux._estop_state_publisher = successful_publisher
        current_time[0] = 15.1
        mux._publish_estop_state()
        assert mux._last_estop_state_publish_error_log_time is None

        mux._estop_state_publisher = failing_publisher
        current_time[0] = 15.2
        mux._publish_estop_state()
        assert len(logger.messages) == 3

        raising_logger = _RecordingLogger(raises=True)
        monkeypatch.setattr(mux, 'get_logger', lambda: raising_logger)
        successful_publisher.publish(Bool())
        mux._estop_state_publisher = successful_publisher
        mux._publish_estop_state()
        mux._estop_state_publisher = failing_publisher

        current_time[0] = 20.0
        mux._publish_estop_state()
        assert len(raising_logger.messages) == 1
        current_time[0] = 21.0
        mux._publish_estop_state()
        assert len(raising_logger.messages) == 1

        decision = mux._core.evaluate(mux._now_sec())
        assert decision.active_source == 'estop'
        assert decision.command == VelocityCommand()

        mux._command_publisher = original_command_publisher
        mux._status_publisher = original_status_publisher
        mux._estop_state_publisher = original_estop_state_publisher
    finally:
        if mux is not None:
            mux.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
