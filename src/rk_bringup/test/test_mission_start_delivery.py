"""双 ACK start 交付的纯软件单元测试。"""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rk_bringup.mission_start_delivery import MissionStartDeliveryState  # noqa: E402


def _route(run_id='run-1', phase='START_STAGE', state='START_STAGE'):
    return {
        'mission_started': True,
        'run_id': run_id,
        'route_phase': phase,
        'state': state,
    }


def _follower(nav_state='START_READY'):
    return {'mission_started': True, 'nav_state': nav_state}


def test_route_and_follower_ack_complete_one_logical_request():
    delivery = MissionStartDeliveryState(3)

    assert delivery.record_transport_publish(2) is True
    delivery.observe_route(_route())
    assert delivery.route_start_ack is True
    assert delivery.follower_start_ack is False
    delivery.observe_follower(_follower())

    assert delivery.start_delivery_completed is True
    assert delivery.as_dict()['logical_request_count'] == 1
    assert delivery.as_dict()['route_run_id'] == 'run-1'


def test_route_ack_rejects_wait_start_and_empty_run_id():
    delivery = MissionStartDeliveryState(3)

    delivery.observe_route(_route(run_id='', phase='WAIT_START'))

    assert delivery.route_start_ack is False
    assert delivery.route_run_id == ''


def test_follower_ack_rejects_wait_start():
    delivery = MissionStartDeliveryState(3)

    delivery.observe_follower(_follower('WAIT_START'))

    assert delivery.follower_start_ack is False


def test_retransmission_does_not_create_second_logical_request_or_change_run_id():
    delivery = MissionStartDeliveryState(3)

    assert delivery.record_transport_publish(2) is True
    assert delivery.record_transport_publish(2) is True
    delivery.observe_route(_route('only-run'))
    delivery.observe_follower(_follower())
    # 同一 run 的后续状态和第三个 transport pulse 均可接受，不会重置 ACK。
    assert delivery.record_transport_publish(2) is False
    delivery.observe_route(_route('only-run', phase='MID_ROUTE'))

    assert delivery.logical_request_count == 1
    assert delivery.route_run_id == 'only-run'
    assert delivery.start_delivery_completed is True
    assert delivery.transport_publish_count == 2


def test_changed_run_id_fails_closed_before_completion():
    delivery = MissionStartDeliveryState(3)

    delivery.observe_route(_route('run-a'))
    delivery.observe_route(_route('run-b'))

    assert delivery.start_delivery_failed is True
    assert delivery.start_delivery_error == 'route_run_id_changed'
    assert delivery.start_delivery_completed is False


def test_publish_limit_and_timeout_fail_closed():
    delivery = MissionStartDeliveryState(1)

    assert delivery.record_transport_publish(2) is True
    assert delivery.record_transport_publish(2) is False
    assert delivery.start_delivery_failed is True
    assert delivery.start_delivery_error == 'transport_publish_limit_reached'

    timed_out = MissionStartDeliveryState(3)
    timed_out.fail_timeout()
    assert timed_out.start_delivery_failed is True
    assert timed_out.start_delivery_error == 'start_delivery_ack_timeout'
