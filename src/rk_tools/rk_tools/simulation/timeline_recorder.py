"""Timeline recorder subscribed only to ROS topics, never the FSM object."""

import csv
import json
import os
import time

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class TimelineRecorder(Node):
    """Write mission timeline and authority evidence from ROS topics."""

    def __init__(self, output_dir, timeline_name):
        super().__init__('national_timeline_recorder')
        os.makedirs(output_dir, exist_ok=True)
        self.output_dir = output_dir
        self.timeline_path = os.path.join(output_dir, timeline_name)
        self.timeline_file = open(self.timeline_path, 'w', newline='', encoding='utf-8')
        self.timeline_writer = csv.writer(self.timeline_file)
        self.timeline_writer.writerow([
            'time_monotonic', 'state', 'event', 'detail', 'final_vx', 'final_wz'
        ])
        self.topic_file = open(os.path.join(output_dir, 'topic_transitions.log'),
                               'a', encoding='utf-8')
        self.action_file = open(os.path.join(output_dir, 'action_calls.log'),
                                'a', encoding='utf-8')
        self.authority_file = open(os.path.join(output_dir, 'control_authority.log'),
                                   'a', encoding='utf-8')
        self.last_state = ''
        self.last_command = (0.0, 0.0)
        self.states = []
        self.events = []
        self.actions = []
        self.create_subscription(String, '/mission/national_state', self._on_state, 10)
        self.create_subscription(String, '/mission/national_events', self._on_event, 10)
        self.create_subscription(String, '/mission/national_actions', self._on_action, 10)
        self.create_subscription(String, '/simulation/national/fake_action_calls',
                                 self._on_fake_action, 10)
        self.create_subscription(String, '/control/cmd_mux_status', self._on_mux, 10)
        self.create_subscription(Twist, '/navigation/cmd_vel', self._on_final, 10)

    def _on_state(self, message):
        try:
            data = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        state = str(data.get('state', ''))
        if state and state != self.last_state:
            self.last_state = state
            self.states.append((time.monotonic(), state, data))
            self._write_timeline(state, 'STATE', json.dumps(data, sort_keys=True))
            self.topic_file.write('[STATE] {}\n'.format(message.data))
            self.topic_file.flush()

    def _on_event(self, message):
        self.events.append(message.data)
        self.topic_file.write('[EVENT] {}\n'.format(message.data))
        self.topic_file.flush()
        try:
            data = json.loads(message.data)
            self._write_timeline(str(data.get('state', self.last_state)),
                                 str(data.get('event', 'EVENT')),
                                 str(data.get('detail', '')))
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def _on_action(self, message):
        self.actions.append(message.data)
        self.action_file.write('[MISSION_ACTION] {}\n'.format(message.data))
        self.action_file.flush()

    def _on_fake_action(self, message):
        self.actions.append(message.data)
        self.action_file.write('[FAKE_ACTION] {}\n'.format(message.data))
        self.action_file.flush()

    def _on_mux(self, message):
        self.authority_file.write('{}\n'.format(message.data))
        self.authority_file.flush()

    def _on_final(self, message):
        self.last_command = (float(message.linear.x), float(message.angular.z))

    def _write_timeline(self, state, event, detail):
        self.timeline_writer.writerow([
            '{:.6f}'.format(time.monotonic()), state, event, detail,
            '{:.6f}'.format(self.last_command[0]),
            '{:.6f}'.format(self.last_command[1]),
        ])
        self.timeline_file.flush()

    def close(self):
        for stream in (self.timeline_file, self.topic_file, self.action_file,
                       self.authority_file):
            if not stream.closed:
                stream.close()

    def destroy_node(self):
        self.close()
        return super().destroy_node()
