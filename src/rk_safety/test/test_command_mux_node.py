from importlib import import_module
from pathlib import Path
import sys
import time
from types import SimpleNamespace
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
Twist = geometry_msgs.Twist
Bool = std_msgs.Bool
SetBool = std_srvs.SetBool


def test_estop_service_logs_delivery_diagnostics_without_changing_response():
    """服务端日志必须区分请求到达与 CLI 响应丢失，且保持 success=true。"""
    node = object.__new__(CommandMuxNode)
    node._core = SimpleNamespace(estop=False)
    log_messages = []

    class Logger:
        def info(self, message):
            log_messages.append(message)

    def transition(enabled):
        previous = bool(node._core.estop)
        node._core.estop = bool(enabled)
        return previous != bool(enabled)

    node._transition_estop = transition
    node.get_logger = lambda: Logger()
    request = SimpleNamespace(data=True)
    response = SimpleNamespace(success=False, message='')

    result = CommandMuxNode._on_estop_service(node, request, response)

    assert result is response
    assert response.success is True
    assert node._core.estop is True
    assert len(log_messages) == 1
    assert 'requested=true previous=false current=true changed=true' in (
        log_messages[0]
    )
    assert 'success=true elapsed_ms=' in log_messages[0]


def test_mission_estop_and_safe_recovery():
    unique_id = uuid.uuid4().hex
    topic_prefix = '/rk_safety_test_{}'.format(unique_id)
    topics = {
        'line': topic_prefix + '/line',
        'mission': topic_prefix + '/mission',
        'locomotion': topic_prefix + '/locomotion',
        'estop': topic_prefix + '/estop',
        'gait_lock': topic_prefix + '/gait_lock',
        'arm_lock': topic_prefix + '/arm_lock',
        'output': topic_prefix + '/output',
        'status': topic_prefix + '/status',
        'estop_service': topic_prefix + '/estop_service',
    }
    ros_args = ['--ros-args']
    parameter_names = {
        'line_cmd_topic': 'line',
        'mission_cmd_topic': 'mission',
        'locomotion_cmd_topic': 'locomotion',
        'estop_topic': 'estop',
        'gait_lock_topic': 'gait_lock',
        'arm_lock_topic': 'arm_lock',
        'output_cmd_topic': 'output',
        'status_topic': 'status',
    }
    for parameter_name, topic_key in parameter_names.items():
        ros_args.extend(
            ['-p', '{}:={}'.format(parameter_name, topics[topic_key])]
        )
    ros_args.extend(['-p', 'control_rate_hz:=40.0'])
    ros_args.extend(['-p', 'enable_estop_service:=true'])
    ros_args.extend(
        ['-p', 'estop_service_name:=' + topics['estop_service']]
    )
    ros_args.extend(
        ['-r', '/navigation/cmd_vel:=' + topic_prefix + '/failsafe_output']
    )
    ros_args.extend(
        ['-r', '/safety/estop:=' + topic_prefix + '/failsafe_estop']
    )

    rclpy.init(args=ros_args)
    mux = None
    probe = None
    executor = None
    try:
        mux = CommandMuxNode()
        probe = rclpy.create_node(
            'rk_safety_test_probe_{}'.format(unique_id[:12])
        )
        outputs = []
        output_subscription = probe.create_subscription(
            Twist,
            topics['output'],
            lambda message: outputs.append(
                (
                    message.linear.x,
                    message.linear.y,
                    message.angular.z,
                )
            ),
            10,
        )
        mission_publisher = probe.create_publisher(
            Twist, topics['mission'], 10
        )
        estop_publisher = probe.create_publisher(Bool, topics['estop'], 10)
        estop_client = probe.create_client(SetBool, topics['estop_service'])
        assert output_subscription is not None

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(mux)
        executor.add_node(probe)

        def spin_until(predicate, timeout_sec=2.0):
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                if predicate():
                    return
            raise AssertionError('Timed out waiting for command mux output')

        def publish_mission(linear_x):
            message = Twist()
            message.linear.x = linear_x
            mission_publisher.publish(message)

        def call_estop(enabled):
            request = SetBool.Request()
            request.data = enabled
            future = estop_client.call_async(request)
            spin_until(future.done)
            response = future.result()
            assert response is not None
            assert response.success is True
            assert response.message
            return response

        spin_until(
            lambda: mission_publisher.get_subscription_count() > 0
            and estop_publisher.get_subscription_count() > 0
            and mux.count_publishers(topics['output']) > 0
            and estop_client.service_is_ready()
        )

        outputs[:] = []
        publish_mission(0.2)
        spin_until(
            lambda: any(abs(output[0] - 0.2) < 1e-9 for output in outputs)
        )

        outputs[:] = []
        response = call_estop(True)
        assert response.message == (
            'Emergency stop enabled; command caches cleared'
        )
        spin_until(
            lambda: len(outputs) >= 3
            and all(output == (0.0, 0.0, 0.0) for output in outputs[-3:])
        )

        response = call_estop(True)
        assert response.message == (
            'Emergency stop already enabled; state unchanged'
        )

        publish_mission(0.25)
        spin_until(lambda: len(outputs) >= 4)
        outputs[:] = []
        response = call_estop(False)
        assert response.message == (
            'Emergency stop cleared; waiting for new command'
        )
        spin_until(
            lambda: len(outputs) >= 3
            and all(
                output == (0.0, 0.0, 0.0)
                for output in outputs[-3:]
            )
        )

        response = call_estop(False)
        assert response.message == (
            'Emergency stop already cleared; state unchanged'
        )

        outputs[:] = []
        publish_mission(0.3)
        spin_until(
            lambda: any(abs(output[0] - 0.3) < 1e-9 for output in outputs)
        )

        outputs[:] = []
        response = call_estop(False)
        assert response.message == (
            'Emergency stop already cleared; state unchanged'
        )
        spin_until(
            lambda: any(abs(output[0] - 0.3) < 1e-9 for output in outputs)
        )

        outputs[:] = []
        estop_message = Bool()
        estop_message.data = True
        estop_publisher.publish(estop_message)
        spin_until(
            lambda: len(outputs) >= 3
            and all(output == (0.0, 0.0, 0.0) for output in outputs[-3:])
        )

        publish_mission(0.35)
        spin_until(lambda: len(outputs) >= 4)
        outputs[:] = []
        estop_message.data = False
        estop_publisher.publish(estop_message)
        spin_until(
            lambda: len(outputs) >= 3
            and all(
                output == (0.0, 0.0, 0.0)
                for output in outputs[-3:]
            )
        )

        outputs[:] = []
        publish_mission(0.4)
        spin_until(
            lambda: any(abs(output[0] - 0.4) < 1e-9 for output in outputs)
        )
    finally:
        if executor is not None:
            if probe is not None:
                executor.remove_node(probe)
            if mux is not None:
                executor.remove_node(mux)
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if mux is not None:
            mux.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
