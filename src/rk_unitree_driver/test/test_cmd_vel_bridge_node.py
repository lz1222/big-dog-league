from rk_unitree_driver.cmd_vel_bridge_node import CmdVelBridgeNode
from rk_unitree_driver.safety_monitor import CommandDecision


class FakeTime:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1_000_000_000)


class FakeLogger:
    def __init__(self):
        self.infos = []

    def info(self, message):
        self.infos.append(message)


class FakeMotionClient:
    def __init__(self):
        self.stop_reasons = []

    def send_stop(self, reason):
        self.stop_reasons.append(reason)


def make_bridge_for_zero_debounce(debounce_sec=0.60):
    node = CmdVelBridgeNode.__new__(CmdVelBridgeNode)
    node.zero_cmd_debounce_time = debounce_sec
    node.stop_publish_min_interval = 0.0
    node._motion_active = True
    node._last_vx = 0.03
    node._last_vyaw = 0.0
    node._zero_cmd_start_time = None
    node._last_stop_publish_time = None
    node._last_suppressed_stop_log_time = None
    node._motion_client = FakeMotionClient()
    logger = FakeLogger()
    node.get_logger = lambda: logger
    node.fake_logger = logger
    return node


def zero_decision():
    return CommandDecision(
        should_stop=True,
        reason='zero velocity command',
    )


def test_short_zero_cmd_is_debounced_after_motion():
    node = make_bridge_for_zero_debounce()

    node._handle_stop_decision(zero_decision(), FakeTime(1.00))
    node._handle_stop_decision(zero_decision(), FakeTime(1.50))

    assert node._motion_active is True
    assert node._motion_client.stop_reasons == []


def test_sustained_zero_cmd_sends_stop_after_debounce():
    node = make_bridge_for_zero_debounce()

    node._handle_stop_decision(zero_decision(), FakeTime(1.00))
    node._handle_stop_decision(zero_decision(), FakeTime(1.61))

    assert node._motion_active is False
    assert node._motion_client.stop_reasons == ['zero velocity command']
