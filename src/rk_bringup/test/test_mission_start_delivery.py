"""双 ACK start 交付的纯软件单元测试。"""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rk_bringup.mission_start_delivery import (  # noqa: E402
    FOLLOWER_NODE_NAME,
    ROUTE_NODE_NAME,
    MissionStartDeliveryState,
    unique_endpoint_records,
)


ROUTE = {'node_name': ROUTE_NODE_NAME, 'node_namespace': '', 'gid': '01'}
FOLLOWER = {
    'node_name': FOLLOWER_NODE_NAME,
    'node_namespace': '',
    'gid': '02',
}
SMOKE = {
    'node_name': 'competition_smoke_publisher',
    'node_namespace': '',
    'gid': '03',
}


def _route(run_id='run-1', phase='START_STAGE', state='START_STAGE'):
    return {
        'mission_started': True,
        'run_id': run_id,
        'route_phase': phase,
        'state': state,
    }


def _follower(nav_state='START_READY'):
    return {'mission_started': True, 'nav_state': nav_state}


def _prepare_delivery(max_publishes=3):
    delivery = MissionStartDeliveryState(max_publishes)
    delivery.observe_start_subscribers([ROUTE, FOLLOWER, SMOKE], 3)
    delivery.observe_status_sources([ROUTE], [FOLLOWER])
    # WAIT_START 是合法预热状态；只有收到来自真实 topic 的新消息才可发送。
    delivery.observe_route({'mission_started': False, 'route_phase': 'WAIT_START'})
    delivery.observe_follower({'mission_started': False, 'nav_state': 'WAIT_START'})
    return delivery


def test_smoke_and_follower_without_route_do_not_permit_publish():
    delivery = MissionStartDeliveryState(3)
    delivery.observe_start_subscribers([FOLLOWER, SMOKE], 2)
    delivery.observe_status_sources([], [FOLLOWER])
    delivery.observe_follower({'mission_started': False, 'nav_state': 'WAIT_START'})

    assert not delivery.ready_to_publish()
    assert delivery.record_transport_publish(2) is False
    assert delivery.required_route_subscriber_discovered is False


def test_smoke_and_route_without_follower_do_not_permit_publish():
    delivery = MissionStartDeliveryState(3)
    delivery.observe_start_subscribers([ROUTE, SMOKE], 2)
    delivery.observe_status_sources([ROUTE], [])
    delivery.observe_route({'mission_started': False, 'route_phase': 'WAIT_START'})

    assert not delivery.ready_to_publish()
    assert delivery.record_transport_publish(2) is False
    assert delivery.required_follower_subscriber_discovered is False


def test_route_and_follower_endpoints_and_preheated_streams_permit_publish():
    delivery = _prepare_delivery()

    assert delivery.ready_to_publish()
    assert delivery.record_transport_publish(3, now=10.0) is True
    assert delivery.transport_publish_count == 1
    assert delivery.start_endpoint_discovery_completed is True
    assert delivery.route_status_stream_observed is True
    assert delivery.follower_status_stream_observed is True


def test_extra_subscribers_cannot_replace_required_nodes():
    delivery = MissionStartDeliveryState(3)
    extra = dict(SMOKE, gid='04')
    delivery.observe_start_subscribers([SMOKE, extra], 2)

    assert delivery.discovered_subscriber_count == 2
    assert delivery.start_endpoint_discovery_completed is False
    assert not delivery.ready_to_publish()


class _Profile:
    reliability = 'RELIABLE'
    durability = 'VOLATILE'


class _Endpoint:
    def __init__(self, name, namespace, gid, profile=None):
        self.node_name = name
        self.node_namespace = namespace
        self.endpoint_gid = gid
        self.qos_profile = profile or _Profile()


def test_same_endpoint_gid_is_deduplicated():
    records = unique_endpoint_records([
        _Endpoint(ROUTE_NODE_NAME, '/', b'\x01'),
        _Endpoint(ROUTE_NODE_NAME, '/', b'\x01'),
        _Endpoint(FOLLOWER_NODE_NAME, '/', b'\x02'),
    ], True)

    assert records is not None
    assert len(records) == 2
    assert {record['gid'] for record in records} == {'01', '02'}


def test_unreadable_endpoint_identity_fails_closed():
    delivery = MissionStartDeliveryState(3)

    delivery.observe_start_subscribers(None, 2)

    assert delivery.start_delivery_failed is True
    assert delivery.start_delivery_error == 'required_start_subscriber_identity_unreadable'


def test_status_streams_must_be_preheated_before_first_publish():
    delivery = MissionStartDeliveryState(3)
    delivery.observe_start_subscribers([ROUTE, FOLLOWER], 2)
    delivery.observe_status_sources([ROUTE], [FOLLOWER])

    assert not delivery.ready_to_publish()
    delivery.observe_route({'mission_started': False, 'route_phase': 'WAIT_START'})
    assert not delivery.ready_to_publish()
    delivery.observe_follower({'mission_started': False, 'nav_state': 'WAIT_START'})
    assert delivery.ready_to_publish()


def test_publish_limit_is_diagnostic_and_delayed_route_ack_can_succeed():
    delivery = _prepare_delivery(3)

    assert delivery.record_transport_publish(3, now=1.0)
    assert delivery.record_transport_publish(3, now=2.0)
    assert delivery.record_transport_publish(3, now=3.0)
    assert delivery.transport_publish_limit_reached is True
    assert delivery.record_transport_publish(3, now=4.0) is False
    assert delivery.start_delivery_failed is False
    delivery.observe_follower(_follower())
    delivery.observe_route(_route())

    assert delivery.start_delivery_completed is True
    assert delivery.start_delivery_failed is False
    assert delivery.logical_request_count == 1


def test_publish_limit_is_diagnostic_and_delayed_follower_ack_can_succeed():
    delivery = _prepare_delivery(1)

    assert delivery.record_transport_publish(2, now=1.0)
    assert delivery.transport_publish_limit_reached is True
    delivery.observe_route(_route())
    delivery.observe_follower(_follower())

    assert delivery.start_delivery_completed is True
    assert delivery.transport_publish_count == 1


def test_deadline_missing_route_or_follower_reports_ack_timeout():
    route_missing = _prepare_delivery()
    route_missing.record_transport_publish(2, now=1.0)
    route_missing.observe_follower(_follower())
    route_missing.fail_timeout()
    assert route_missing.start_delivery_error == 'start_delivery_ack_timeout'
    assert route_missing.missing_ack_nodes == [ROUTE_NODE_NAME]

    follower_missing = _prepare_delivery()
    follower_missing.record_transport_publish(2, now=1.0)
    follower_missing.observe_route(_route())
    follower_missing.fail_timeout()
    assert follower_missing.start_delivery_error == 'start_delivery_ack_timeout'
    assert follower_missing.missing_ack_nodes == [FOLLOWER_NODE_NAME]


def test_required_endpoint_lost_before_ack_fails_closed():
    delivery = _prepare_delivery()
    delivery.record_transport_publish(2, now=1.0)

    delivery.observe_start_subscribers([FOLLOWER, SMOKE], 2)

    assert delivery.start_delivery_failed is True
    assert delivery.start_delivery_error == 'required_start_subscriber_lost'


def test_retransmission_does_not_create_second_run_id_and_changed_run_id_fails():
    delivery = _prepare_delivery()
    delivery.record_transport_publish(2, now=1.0)
    delivery.observe_route(_route('only-run'))
    delivery.observe_follower(_follower())
    delivery.observe_route(_route('only-run', phase='MID_ROUTE'))

    assert delivery.logical_request_count == 1
    assert delivery.route_run_id == 'only-run'
    assert delivery.start_delivery_completed is True

    changing = _prepare_delivery()
    changing.observe_route(_route('run-a'))
    changing.observe_route(_route('run-b'))
    assert changing.start_delivery_error == 'route_run_id_changed'
