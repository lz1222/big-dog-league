#!/usr/bin/env python3

import json
import time

try:
    from unitree_api.msg import Request
except ModuleNotFoundError as import_error:
    Request = None
    UNITREE_IMPORT_ERROR = import_error


class Go2MotionClient:
    """Publish Unitree Go2 Sport Request messages."""

    STOP_MOVE_API_ID = 1003
    MOVE_API_ID = 1008

    def __init__(self, node, sport_request_topic):
        if Request is None:
            raise RuntimeError(
                'unitree_api.msg.Request is not available. '
                'Source the Unitree ROS2 workspace before running '
                'rk_unitree_driver.'
            ) from UNITREE_IMPORT_ERROR

        self._node = node
        self._publisher = node.create_publisher(
            Request,
            sport_request_topic,
            10
        )
        self._logger = node.get_logger()
        self._clock = node.get_clock()
        self._sport_request_topic = sport_request_topic

        self._logger.info(
            f'Go2 Sport request publisher ready: {sport_request_topic}'
        )

    def send_move(self, vx, vyaw):
        request = self._make_request(self.MOVE_API_ID)
        parameter = {
            'x': float(vx),
            'y': 0.0,
            'z': float(vyaw),
        }
        request.parameter = json.dumps(parameter, separators=(',', ':'))
        self._publisher.publish(request)

        self._logger.info(
            '[' + self._timestamp() + '] '
            f'Move api_id={self.MOVE_API_ID} '
            f'vx={float(vx):.3f}, vy=0.000, '
            f'vyaw={float(vyaw):.3f}'
        )

    def send_stop(self, reason):
        request = self._make_request(self.STOP_MOVE_API_ID)
        request.parameter = ''
        self._publisher.publish(request)

        self._logger.warn(
            '[' + self._timestamp() + '] '
            f'StopMove api_id={self.STOP_MOVE_API_ID} '
            f'reason="{reason}"'
        )

    def send_repeated_stop(self, reason, count, period_sec):
        repeat_count = max(1, int(count))
        sleep_sec = max(0.0, float(period_sec))

        for index in range(repeat_count):
            self.send_stop(f'{reason} ({index + 1}/{repeat_count})')
            if index + 1 < repeat_count and sleep_sec > 0.0:
                time.sleep(sleep_sec)

    def _make_request(self, api_id):
        request = Request()
        request.header.identity.api_id = int(api_id)
        return request

    def _timestamp(self):
        stamp = self._clock.now().to_msg()
        return f'{stamp.sec}.{stamp.nanosec:09d}'
