#!/usr/bin/env python3

import json
import time


class Go2MotionClient:
    """Publish Unitree Go2 Sport Request messages."""

    MOCK_BACKEND = 'mock'
    UNITREE_ROS2_BACKEND = 'unitree_ros2'
    SUPPORTED_BACKENDS = {
        MOCK_BACKEND,
        UNITREE_ROS2_BACKEND,
    }

    STOP_MOVE_API_ID = 1003
    STAND_DOWN_API_ID = 1005
    MOVE_API_ID = 1008
    DAMP_API_ID = 1001

    def __init__(self, node, sport_request_topic, backend):
        if backend not in self.SUPPORTED_BACKENDS:
            supported = ', '.join(sorted(self.SUPPORTED_BACKENDS))
            raise ValueError(f'backend must be one of: {supported}')

        self._node = node
        self._logger = node.get_logger()
        self._clock = node.get_clock()
        self._sport_request_topic = sport_request_topic
        self._backend = backend
        self._publisher = None
        self._request_type = None

        if self._backend == self.UNITREE_ROS2_BACKEND:
            self._setup_unitree_ros2_publisher()
        else:
            self._logger.info(
                'Go2 mock backend ready: '
                'logging Sport commands without unitree_api publisher'
            )

    def _setup_unitree_ros2_publisher(self):
        try:
            from unitree_api.msg import Request
        except ModuleNotFoundError as import_error:
            raise RuntimeError(
                'unitree_api.msg.Request is not available. '
                'Source the Unitree ROS2 workspace before running '
                'rk_unitree_driver with backend:=unitree_ros2.'
            ) from import_error

        self._request_type = Request
        self._publisher = self._node.create_publisher(
            self._request_type,
            self._sport_request_topic,
            10
        )

        self._logger.info(
            'Go2 Sport request publisher ready: '
            f'{self._sport_request_topic}'
        )

    def send_move(self, vx, vyaw):
        if self._backend == self.UNITREE_ROS2_BACKEND:
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
        if self._backend == self.UNITREE_ROS2_BACKEND:
            request = self._make_request(self.STOP_MOVE_API_ID)
            request.parameter = ''
            self._publisher.publish(request)

        self._logger.warn(
            '[' + self._timestamp() + '] '
            f'StopMove api_id={self.STOP_MOVE_API_ID} '
            f'reason="{reason}"'
        )

    def send_stand_down(self, reason):
        if self._backend == self.UNITREE_ROS2_BACKEND:
            request = self._make_request(self.STAND_DOWN_API_ID)
            request.parameter = ''
            self._publisher.publish(request)

        self._logger.warn(
            '[' + self._timestamp() + '] '
            f'StandDown api_id={self.STAND_DOWN_API_ID} '
            f'reason="{reason}"'
        )

    def send_damp(self, reason):
        if self._backend == self.UNITREE_ROS2_BACKEND:
            request = self._make_request(self.DAMP_API_ID)
            request.parameter = ''
            self._publisher.publish(request)

        self._logger.warn(
            '[' + self._timestamp() + '] '
            f'Damp api_id={self.DAMP_API_ID} '
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
        request = self._request_type()
        request.header.identity.api_id = int(api_id)
        return request

    def _timestamp(self):
        stamp = self._clock.now().to_msg()
        return f'{stamp.sec}.{stamp.nanosec:09d}'
