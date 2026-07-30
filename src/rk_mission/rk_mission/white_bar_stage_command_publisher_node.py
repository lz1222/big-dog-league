#!/usr/bin/env python3

"""Publish explicit white-bar stage commands from mission milestones."""

import json
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from rk_mission.white_bar_stage_command_core import (
    WhiteBarStageCommandSequencer,
)


def decode_json_object(raw_message):
    """Decode one String payload into a JSON object, or return None."""
    if type(raw_message) is not str:
        return None
    try:
        payload = json.loads(raw_message)
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    return payload if type(payload) is dict else None


class WhiteBarStageCommandPublisherNode(Node):
    """Adapt sequencer input and output events to the configured ROS topics."""

    def __init__(self):
        super().__init__('white_bar_stage_command_publisher')
        self._declare_parameters()
        self._read_parameters()
        self.sequencer = WhiteBarStageCommandSequencer(
            command_retry_sec=self.command_retry_sec,
            command_ack_timeout_sec=self.command_ack_timeout_sec,
            max_command_retries=self.max_command_retries,
            finish_milestone_state=self.finish_milestone_state,
        )

        self.command_publisher = self.create_publisher(
            String,
            self.white_bar_stage_command_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            self.publisher_status_topic,
            10,
        )
        self.create_subscription(
            Bool,
            self.mission_start_topic,
            self._on_mission_start,
            10,
        )
        self.create_subscription(
            Bool,
            self.mission_stop_topic,
            self._on_mission_stop,
            10,
        )
        self.create_subscription(
            String,
            self.white_bar_stage_status_topic,
            self._on_stage_status,
            10,
        )
        self.create_subscription(
            String,
            self.line_course_state_topic,
            self._on_line_course_state,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self._on_timer,
        )
        self._handle_event(
            self.sequencer.status_event('stage_command_publisher_ready')
        )
        self.get_logger().info(
            'White-bar stage command publisher ready: '
            f'command_topic={self.white_bar_stage_command_topic}, '
            f'milestone={self.finish_milestone_state}'
        )

    def _declare_parameters(self):
        topic_defaults = {
            'mission_start_topic': '/mission/start',
            'mission_stop_topic': '/mission/stop',
            'white_bar_stage_status_topic': '/mission/white_bar_stage_status',
            'white_bar_stage_command_topic': '/mission/white_bar_stage_command',
            'line_course_state_topic': '/mission/line_course_state',
            'publisher_status_topic': (
                '/mission/white_bar_stage_command_publisher_status'
            ),
        }
        for name, value in topic_defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('finish_milestone_state', 'TURN_AFTER_RED')
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('command_retry_sec', 0.5)
        self.declare_parameter('command_ack_timeout_sec', 5.0)
        self.declare_parameter('max_command_retries', 5)

    def _read_parameters(self):
        topic_names = (
            'mission_start_topic',
            'mission_stop_topic',
            'white_bar_stage_status_topic',
            'white_bar_stage_command_topic',
            'line_course_state_topic',
            'publisher_status_topic',
            'finish_milestone_state',
        )
        for name in topic_names:
            value = str(self.get_parameter(name).value).strip()
            if not value:
                raise ValueError(f'{name} must not be empty')
            setattr(self, name, value)
        self.control_rate_hz = self._positive_float_parameter(
            'control_rate_hz'
        )
        self.command_retry_sec = self._positive_float_parameter(
            'command_retry_sec'
        )
        self.command_ack_timeout_sec = self._positive_float_parameter(
            'command_ack_timeout_sec'
        )
        max_retries = self.get_parameter('max_command_retries').value
        if type(max_retries) is not int or max_retries < 0:
            raise ValueError('max_command_retries must be a nonnegative integer')
        self.max_command_retries = max_retries

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _on_mission_start(self, msg):
        if msg.data:
            self._handle_event(self.sequencer.mission_start(time.monotonic()))

    def _on_mission_stop(self, msg):
        if msg.data:
            self._handle_event(self.sequencer.mission_stop(time.monotonic()))

    def _on_stage_status(self, msg):
        self._handle_event(
            self.sequencer.on_stage_status(
                decode_json_object(msg.data),
                time.monotonic(),
            )
        )

    def _on_line_course_state(self, msg):
        self._handle_event(
            self.sequencer.on_line_course_state(
                decode_json_object(msg.data),
                time.monotonic(),
            )
        )

    def _on_timer(self):
        self._handle_event(self.sequencer.on_timer(time.monotonic()))

    def _handle_event(self, event):
        if event.action == 'SEND_COMMAND':
            command = String()
            command.data = json.dumps(
                event.command_payload,
                separators=(',', ':'),
            )
            self.command_publisher.publish(command)
        if event.action == 'NONE':
            return
        status = String()
        status.data = json.dumps({
            'state': event.state,
            'run_id': event.run_id,
            'sequence': event.sequence,
            'requested_stage': event.requested_stage,
            'retry_count': event.retry_count,
            'start_completed': event.start_completed,
            'finish_milestone_seen': event.finish_milestone_seen,
            'finish_completed': event.finish_completed,
            'reason': event.reason,
        }, separators=(',', ':'))
        self.status_publisher.publish(status)
        if event.action == 'FAULT':
            self.get_logger().error(
                f'[WHITE_BAR_STAGE_COMMAND] {event.state}: {event.reason}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = WhiteBarStageCommandPublisherNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
