#!/usr/bin/env python3
"""Run national integrated route scenarios through real ROS topics/Actions.

This process instantiates the existing line follower and command mux, the new
national mission node, and only simulation perception/action endpoints.  No
camera, robot bridge, SportClient, or SDK action executable is launched.
"""

import argparse
import json
import os
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float32, String

from rk_interfaces.action import RunMission
from rk_interfaces.msg import ItemTag, ItemTagArray, LineTrack
from rk_interfaces.msg import SignDetection, SignDetectionArray
from rk_mission.national_mission_node import NationalMissionNode
from rk_navigation.line_follower_node import LineFollowerNode
from rk_safety.command_mux_node import CommandMuxNode

from .simulation.fake_subtask_servers import FakeSubtaskServers
from .simulation.scenario_definitions import (
    ALL_SCENARIOS,
    FAULT_SCENARIOS,
    get_scenario,
)
from .simulation.timeline_recorder import TimelineRecorder


TERMINAL_STATES = {'MISSION_COMPLETE', 'SAFE_STOP', 'ESTOP', 'MISSION_FAILED'}


@dataclass
class ScenarioResult:
    name: str
    terminal_state: str
    expected_terminal: str
    passed: bool
    duration_sec: float
    failure_code: str = ''
    failure_reason: str = ''
    action_count: int = 0
    nonzero_authority_conflict: bool = False


def _parameter_overrides(values):
    return [Parameter(name, value=value) for name, value in values.items()]


def simulation_parameters():
    """Fast DEVELOPMENT DEFAULT overrides used only by no-hardware suite."""
    values = {
        'simulation_mode': True,
        'control_rate_hz': 40.0,
        'min_line_confidence': 0.50,
        'min_effective_speed': 0.01,
        'final_cmd_stale_sec': 3.00,
        'zero_confirm_frames': 2,
        # Do not accept messages queued before an Action result/state change
        # as a new post-action line reacquisition in the multithreaded suite.
        'line_reacquire_frames': 5,
        'white_confirm_frames': 2,
        'marker_confirm_frames': 2,
        'sign_confirm_frames': 2,
        'red_confirm_frames': 2,
        # These are simulation-only scheduler allowances.  They are longer
        # than individual fake-server delays so duplicate-result coverage
        # cannot be misclassified as an Action transport timeout.
        'state_timeout_sec': 10.00,
        'action_timeout_sec': 5.00,
        'reacquire_timeout_sec': 2.00,
        'pattern_search_timeout_sec': 5.00,
        'sign_search_timeout_sec': 5.00,
        'turn_timeout_sec': 3.00,
        'mission_turn_speed': 0.85,
        'search_turn_speed': 0.55,
        'pick_turn_target_rad': 0.16,
        'inspection_turn_target_rad': 0.16,
        'search_small_angle_rad': 0.06,
        'search_medium_angle_rad': 0.10,
        'search_large_angle_rad': 0.14,
        'search_settle_sec': 0.03,
        'red_track_speed': 0.25,
        'post_red_marker_distance': 0.025,
        'post_red_marker_time_fallback': 0.08,
        'post_red_marker_hard_timeout': 2.00,
        'pick_arc_min_distance': 0.015,
        'pick_arc_target_distance': 0.030,
        # Command distance is an estimate in this no-odometry simulator.  Its
        # upper bound only guards obviously runaway virtual routes; it must
        # tolerate a delayed host callback that crosses the small target in
        # one integration step.
        'pick_arc_max_distance': 1.000,
        'pick_arc_min_curve_duration': 0.0,
        'pick_arc_min_abs_angular_z': 0.03,
        'pick_arc_target_turn_progress_rad': 0.0,
        'pick_arc_hard_timeout': 5.00,
        # A one-frame marker coincident with state entry is noise, not a
        # crossing.  Normal scenarios continue publishing after this window.
        'start_white_min_segment_sec': 0.10,
        'finish_white_min_segment_sec': 0.10,
    }
    for prefix in (
        'maze_entry', 'stairs_approach', 'transfer_platform',
        'place_platform', 'final_zone',
    ):
        values.update({
            prefix + '_min_distance': 0.010,
            prefix + '_target_distance': 0.025,
            prefix + '_max_distance': 1.000,
            prefix + '_min_effective_time': 0.020,
            prefix + '_target_effective_time': 0.075,
            prefix + '_hard_timeout': 3.00,
        })
    return values


class NationalScenarioDriver(Node):
    """Drive one scenario strictly through public ROS topics and Actions."""

    def __init__(self, scenario):
        super().__init__('national_route_scenario_driver')
        self.scenario = scenario
        self.started = False
        self.start_time = time.monotonic()
        self.state = 'WAITING_FOR_NODE'
        self.state_enter_time = self.start_time
        self.state_payload = {}
        self.done = False
        self.terminal_state = ''
        self.failure_code = ''
        self.failure_reason = ''
        self.goal_result = None
        self.run_index = 0
        self.second_start_at = None
        self.white_noise_sent = False
        self.red_noise_sent = False
        self.estop_sent = False
        self.nonfinite_sent = False
        self.suppress_sent = False
        self.override_sent = False
        self.start_reacquire_fault_armed = False
        self.marker_publish_count = 0
        self.sign_publish_count = 0
        self.repeated_start_sent = False
        self.action_count = 0
        self.line_pub = self.create_publisher(LineTrack, '/perception/line_track', 10)
        self.white_pub = self.create_publisher(Bool, '/perception/route_markers/white_line', 10)
        self.white_confidence_pub = self.create_publisher(
            Float32, '/perception/route_markers/white_line_confidence', 10
        )
        self.red_pub = self.create_publisher(PointStamped, '/perception/route_markers/red_circle', 10)
        self.item_pub = self.create_publisher(ItemTagArray, '/perception/item_tags', 10)
        self.sign_pub = self.create_publisher(SignDetectionArray, '/perception/sign_detections', 10)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.candidate_pub = self.create_publisher(Twist, '/control/mission_cmd', 10)
        self.control_pub = self.create_publisher(String, '/mission/national_control', 10)
        self.legacy_start_pub = self.create_publisher(Bool, '/mission/start', 10)
        self.action_override_pub = self.create_publisher(
            String, '/simulation/national/action_result_override', 10
        )
        self.suppress_final_pub = self.create_publisher(
            Bool, '/simulation/national/suppress_final_cmd', 10
        )
        self.run_client = ActionClient(self, RunMission, '/mission/run')
        self.create_subscription(String, '/mission/national_state', self._on_state, 10)
        self.create_subscription(String, '/mission/national_actions', self._on_action, 10)
        self.timer = self.create_timer(0.02, self._on_timer)

    def _on_timer(self):
        now = time.monotonic()
        if not self.started:
            if self.run_client.server_is_ready():
                self._send_goal()
            return
        if self.second_start_at is not None and now >= self.second_start_at:
            self.second_start_at = None
            self._send_goal()
        if self.done:
            return
        self._publish_line()
        self._publish_white()
        self._publish_red()
        self._publish_marker()
        self._publish_sign()
        self._inject_faults()

    def _send_goal(self):
        goal = RunMission.Goal()
        goal.start = True
        self.started = True
        future = self.run_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        if self.scenario.fault == 'action_and_json_repeat_start' and not self.repeated_start_sent:
            self.repeated_start_sent = True
            duplicate = self.run_client.send_goal_async(goal)
            duplicate.add_done_callback(lambda completed: completed.result())
            message = Bool()
            message.data = True
            self.legacy_start_pub.publish(message)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception:
            return
        if handle is None or not handle.accepted:
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        try:
            result = future.result().result
            self.goal_result = (bool(result.success), str(result.message))
        except Exception as exc:
            self.goal_result = (False, 'goal result exception: {}'.format(exc))

    def _on_state(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        state = str(payload.get('state', ''))
        now = time.monotonic()
        if state != self.state:
            self.state = state
            self.state_enter_time = now
        self.state_payload = payload
        if state in TERMINAL_STATES:
            if (
                self.scenario.fault == 'second_context_reset'
                and self.run_index == 0
                and state == 'MISSION_COMPLETE'
            ):
                self.run_index = 1
                reset = String()
                reset.data = 'reset'
                self.control_pub.publish(reset)
                self.second_start_at = now + 0.15
                return
            self.terminal_state = state
            self.failure_code = str(payload.get('failure_code', ''))
            self.failure_reason = str(payload.get('failure_reason', ''))
            self.done = True

    def _on_action(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if payload.get('phase') == 'sent':
            self.action_count += 1
            if (
                self.scenario.fault == 'jump_line_reacquire_timeout'
                and payload.get('task_name') == 'start_jump'
            ):
                # Stop publishing valid line data before the Action result.
                # This drains pre-jump samples before START_LINE_REACQUIRE can
                # begin accepting its own consecutive frames.
                self.start_reacquire_fault_armed = True
        if (
            self.scenario.fault == 'action_duplicate_result'
            and payload.get('phase') == 'accepted'
            and not self.override_sent
        ):
            self.override_sent = True
            override = String()
            override.data = json.dumps({
                'token': payload['token'], 'success': True,
                'message': 'injected first fake result',
                'task_name': payload.get('task_name', ''),
            })
            self.action_override_pub.publish(override)

    def _line_is_visible(self):
        if (
            self.scenario.fault == 'jump_line_reacquire_timeout'
            and self.start_reacquire_fault_armed
        ):
            return False
        if self.scenario.fault == 'state_timeout' and self.state == 'STAIRS_APPROACH':
            return False
        if self.scenario.fault == 'red_post_offset_timeout' and self.state == 'RED_CIRCLE_POST_OFFSET':
            return False
        return True

    def _publish_line(self):
        msg = LineTrack()
        msg.line_visible = self._line_is_visible()
        msg.confidence = 0.95 if msg.line_visible else 0.0
        msg.lateral_error = 0.0
        msg.heading_error = -0.50 if (
            self.state == 'PICK_ARC_APPROACH'
            and self.scenario.fault != 'pick_arc_not_reached'
        ) else 0.0
        self.line_pub.publish(msg)

    def _publish_white(self):
        message = Bool()
        active = self.state in ('START_WHITE_LINE_CONFIRM', 'FINISH_WHITE_LINE_CONFIRM')
        if self.scenario.fault == 'start_white_single_noise' and self.state == 'START_WHITE_LINE_CONFIRM':
            message.data = not self.white_noise_sent
            self.white_noise_sent = True
        else:
            message.data = active
        confidence = Float32()
        confidence.data = 0.95 if message.data else 0.0
        self.white_confidence_pub.publish(confidence)
        self.white_pub.publish(message)

    def _publish_red(self):
        message = PointStamped()
        visible = False
        y = 0.0
        if self.state == 'INSPECTION_APPROACH':
            if self.scenario.fault == 'red_single_noise':
                visible = not self.red_noise_sent
                self.red_noise_sent = True
                y = 0.45
            else:
                visible = True
                y = 0.45
        elif self.state == 'RED_CIRCLE_TRACK':
            visible = True
            y = 0.82
        elif self.scenario.fault == 'red_early_ignored' and self.state == 'PICK_ARC_APPROACH':
            visible = True
            y = 0.82
        message.point.x = 0.5
        message.point.y = y
        message.point.z = 0.90 if visible else 0.0
        self.red_pub.publish(message)

    def _marker_id(self):
        if self.run_index == 1:
            return 2
        if self.scenario.fault == 'place_target_missing':
            return 3
        if self.scenario.fault == 'marker_alternates':
            return 1 if self.marker_publish_count % 2 else 2
        return self.scenario.marker_id

    def _publish_marker(self):
        in_marker_state = self.state in ('PICK_PATTERN_SEARCH', 'PICK_PATTERN_CONFIRM')
        repeat_after_latch = self.scenario.fault == 'marker_repeat_after_latch' and self.state in ('ARM_PICK_FAKE', 'PICK_TURN_RIGHT')
        if not in_marker_state and not repeat_after_latch:
            return
        if self.scenario.fault == 'pick_pattern_not_found':
            return
        array = ItemTagArray()
        self.marker_publish_count += 1
        if self.scenario.fault == 'marker_alternates':
            # A single perception cycle reports mutually inconsistent IDs.
            # Feeding both in order is deterministic under a multithreaded
            # executor and must keep the consecutive confirmation gate reset.
            array.tags = []
            for marker_id in (1, 2):
                item = ItemTag()
                item.tag_id = marker_id
                item.item_type = 'place_marker'
                item.confidence = 0.95
                array.tags.append(item)
        else:
            item = ItemTag()
            item.tag_id = self._marker_id()
            item.item_type = 'place_marker'
            item.confidence = 0.95
            array.tags = [item]
        self.item_pub.publish(array)

    def _publish_sign(self):
        if self.state != 'INSPECTION_SIGN_CONFIRM' or self.scenario.fault == 'inspection_sign_missing':
            return
        values = ('electric_shock', 'oxidizer', 'radiation')
        value = self.scenario.inspection_type
        if self.scenario.fault == 'inspection_sign_alternates':
            value = values[self.sign_publish_count % len(values)]
        self.sign_publish_count += 1
        detection = SignDetection()
        detection.sign_type = 'warning_sign'
        detection.sign_value = value
        detection.confidence = 0.95
        array = SignDetectionArray()
        array.detections = [detection]
        self.sign_pub.publish(array)

    def _inject_faults(self):
        if self.scenario.fault == 'mid_route_estop' and self.state == 'TRANSFER_PLATFORM_APPROACH' and not self.estop_sent:
            self.estop_sent = True
            message = Bool()
            message.data = True
            self.estop_pub.publish(message)
        if self.scenario.fault == 'final_cmd_stale' and self.state == 'PICK_TURN_LEFT' and not self.suppress_sent:
            self.suppress_sent = True
            message = Bool()
            message.data = True
            self.suppress_final_pub.publish(message)
        if self.scenario.fault == 'nonfinite_mission_candidate' and self.state == 'STAIRS_APPROACH' and not self.nonfinite_sent:
            self.nonfinite_sent = True
            message = Twist()
            message.linear.x = float('nan')
            self.candidate_pub.publish(message)


def _timeline_name(scenario):
    return '{}_timeline.csv'.format(scenario.name) if scenario.fault else 'nominal_{}_timeline.csv'.format(scenario.name)


def run_scenario(scenario_name, output_dir, timeout_sec=None):
    """Create and tear down one complete no-hardware ROS graph."""
    scenario = get_scenario(scenario_name)
    os.makedirs(output_dir, exist_ok=True)
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=10)
    spin_thread = None
    recorder = None
    nodes = []
    started = time.monotonic()
    try:
        mission_parameters = simulation_parameters()
        if scenario.fault == 'pick_arc_not_reached':
            mission_parameters['pick_arc_hard_timeout'] = 0.45
        if scenario.fault == 'maze_timeout':
            mission_parameters['action_timeout_sec'] = 0.45
        if scenario.fault == 'jump_line_reacquire_timeout':
            mission_parameters['reacquire_timeout_sec'] = 0.55
        if scenario.fault in ('start_white_single_noise', 'state_timeout'):
            mission_parameters['state_timeout_sec'] = 1.00
        if scenario.fault in ('pick_pattern_not_found', 'marker_alternates'):
            mission_parameters['pattern_search_timeout_sec'] = 1.00
        if scenario.fault in ('inspection_sign_missing', 'inspection_sign_alternates'):
            mission_parameters['sign_search_timeout_sec'] = 1.00
        if scenario.fault == 'red_single_noise':
            mission_parameters['state_timeout_sec'] = 1.00
        if scenario.fault == 'red_post_offset_timeout':
            mission_parameters['post_red_marker_hard_timeout'] = 0.50
        if scenario.fault == 'final_cmd_stale':
            mission_parameters['final_cmd_stale_sec'] = 0.50
        mission = NationalMissionNode(_parameter_overrides(mission_parameters))
        follower = LineFollowerNode(parameter_overrides=_parameter_overrides({
            'suggested_cmd_topic': '/control/line_cmd',
            'mission_start_topic': '/mission/national_line/start',
            'mission_stop_topic': '/mission/national_line/stop',
            'control_rate_hz': 40.0,
            'base_speed': 0.35,
            'min_driving_speed': 0.20,
            'mid_speed': 0.25,
            'slow_speed': 0.20,
            'line_follow_min_confidence': 0.50,
            'debug_log': False,
        }))
        mux = CommandMuxNode()
        fake_servers = FakeSubtaskServers(scenario)
        driver = NationalScenarioDriver(scenario)
        recorder = TimelineRecorder(output_dir, _timeline_name(scenario))
        nodes = [mission, follower, mux, fake_servers, driver, recorder]
        for node in nodes:
            executor.add_node(node)
        # ``spin_once`` only dispatches one ready callback.  This graph has
        # several 40-50 Hz publishers, so a periodic spin_once loop can starve
        # marker/reacquire callbacks and turn timing into a host-load test.
        # Keep ROS continuously spinning while the scenario watchdog waits.
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        limit = timeout_sec if timeout_sec is not None else (45.0 if not scenario.fault else 30.0)
        if scenario.fault == 'second_context_reset':
            limit = 70.0
        deadline = time.monotonic() + limit
        while time.monotonic() < deadline and not driver.done:
            time.sleep(0.01)
        if not driver.done:
            driver.terminal_state = 'TIMEOUT'
            driver.failure_code = 'SIMULATION_TIMEOUT'
            driver.failure_reason = 'scenario wall timeout'
        time.sleep(0.15)
        terminal = driver.terminal_state or 'TIMEOUT'
        conflict = _authority_conflict(recorder)
        return ScenarioResult(
            scenario.name, terminal, scenario.expected_terminal,
            terminal == scenario.expected_terminal, time.monotonic() - started,
            driver.failure_code, driver.failure_reason, driver.action_count, conflict,
        )
    finally:
        executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        if recorder is not None:
            recorder.close()
        for node in reversed(nodes):
            try:
                executor.remove_node(node)
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


def _authority_conflict(recorder):
    """Nonzero final command during estop/lock is a control-authority conflict."""
    try:
        with open(os.path.join(recorder.output_dir, 'control_authority.log'), encoding='utf-8') as stream:
            for line in stream:
                data = json.loads(line)
                locked = bool(data.get('estop') or data.get('gait_lock') or data.get('arm_lock'))
                moving = abs(float(data.get('final_vx', 0.0))) > 0.01 or abs(float(data.get('final_wz', 0.0))) > 0.01
                if locked and moving:
                    return True
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return True
    return False


def write_static_artifacts(output_dir):
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, 'mission_state_diagram.md'), 'w', encoding='utf-8') as stream:
        stream.write('# National integrated mission v1 state diagram\n\n```mermaid\nstateDiagram-v2\n')
        stream.write('WAIT_START --> START_LINE_FOLLOW --> START_WHITE_LINE_CONFIRM --> START_JUMP --> START_LINE_REACQUIRE\n')
        stream.write('START_LINE_REACQUIRE --> MAZE_ENTRY_APPROACH --> MAZE_TRAVERSE_FAKE --> MAZE_EXIT_REACQUIRE\n')
        stream.write('MAZE_EXIT_REACQUIRE --> STAIRS_APPROACH --> STAIRS_TRAVERSE_FAKE --> STAIRS_EXIT_REACQUIRE\n')
        stream.write('STAIRS_EXIT_REACQUIRE --> PICK_ARC_APPROACH --> PICK_ARC_APEX_STOP --> PICK_TURN_LEFT --> PICK_PATTERN_SEARCH --> PICK_PATTERN_CONFIRM --> ARM_PICK_FAKE --> PICK_TURN_RIGHT --> PICK_LINE_REACQUIRE\n')
        stream.write('PICK_LINE_REACQUIRE --> TRANSFER_PLATFORM_APPROACH --> TRANSFER_STOP --> ARM_TRANSFER_PLACE_FAKE --> ARM_TRANSFER_PICK_FAKE --> TRANSFER_LINE_REACQUIRE\n')
        stream.write('TRANSFER_LINE_REACQUIRE --> INSPECTION_APPROACH --> RED_CIRCLE_TRACK --> RED_CIRCLE_POST_OFFSET --> INSPECTION_STOP --> INSPECTION_TURN_LEFT --> INSPECTION_SIGN_CONFIRM --> INSPECTION_ACTION --> INSPECTION_TURN_RIGHT --> INSPECTION_LINE_REACQUIRE\n')
        stream.write('INSPECTION_LINE_REACQUIRE --> PLACE_PLATFORM_APPROACH --> PLACE_PLATFORM_STOP --> ARM_PLACE_SELECTED_FAKE --> RETURN_LINE_FOLLOW --> FINISH_WHITE_LINE_CONFIRM --> FINISH_JUMP --> FINISH_LINE_REACQUIRE --> FINAL_ZONE_APPROACH --> FINAL_STOP --> MISSION_COMPLETE\n')
        stream.write('state safety { PAUSED SAFE_STOP RECOVERY ESTOP MISSION_FAILED }\n```\n')
    with open(os.path.join(output_dir, 'failures_fixed.md'), 'w', encoding='utf-8') as stream:
        stream.write('# Automatically guarded conditions\n\n- Duplicate action results are ignored by token.\n- Invalid command mux candidates fail closed.\n- White-line acceptance is state scoped.\n- Alternating marker/sign inputs reset confirmation.\n')
    with open(os.path.join(output_dir, 'remaining_field_unknowns.md'), 'w', encoding='utf-8') as stream:
        stream.write('# Remaining field unknowns\n\n- All parameters are DEVELOPMENT DEFAULT / NOT FIELD VALIDATED.\n- Commanded distance is not odometry.\n- Maze/stairs/arm tasks are fake adapters here.\n- Jump result keeps physical_crossing_unverified=true.\n- No real SDK, robot, camera, FrontJump, stairs, or arm is called.\n')


def _write_suite_summary(output_dir, results):
    with open(os.path.join(output_dir, 'scenario_summary.md'), 'w', encoding='utf-8') as stream:
        stream.write('# National integrated mission simulation summary\n\n| scenario | terminal | expected | pass | duration sec | actions | conflict |\n| --- | --- | --- | --- | ---: | ---: | --- |\n')
        for result in results:
            stream.write('| {0.name} | {0.terminal_state} | {0.expected_terminal} | {0.passed} | {0.duration_sec:.3f} | {0.action_count} | {0.nonzero_authority_conflict} |\n'.format(result))
    with open(os.path.join(output_dir, 'fault_scenarios.csv'), 'w', encoding='utf-8') as stream:
        stream.write('scenario,terminal,expected,passed,failure_code,failure_reason\n')
        fault_names = {scenario.name for scenario in FAULT_SCENARIOS}
        for result in results:
            if result.name in fault_names:
                stream.write('{},{},{},{},{},{}\n'.format(result.name, result.terminal_state, result.expected_terminal, result.passed, result.failure_code.replace(',', ';'), result.failure_reason.replace(',', ';')))


def write_scenario_result(output_dir, result):
    """Persist one child-process result for the isolated suite harness."""
    results_dir = os.path.join(output_dir, 'scenario_results')
    os.makedirs(results_dir, exist_ok=True)
    path = os.path.join(results_dir, '{}.json'.format(result.name))
    payload = {
        'name': result.name,
        'terminal_state': result.terminal_state,
        'expected_terminal': result.expected_terminal,
        'passed': result.passed,
        'duration_sec': result.duration_sec,
        'failure_code': result.failure_code,
        'failure_reason': result.failure_reason,
        'action_count': result.action_count,
        'nonzero_authority_conflict': result.nonzero_authority_conflict,
    }
    with open(path, 'w', encoding='utf-8') as stream:
        json.dump(payload, stream, sort_keys=True)


def default_output_dir():
    return '/tmp/national_integrated_sim_{}'.format(time.strftime('%Y%m%d_%H%M%S'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', default='marker1_radiation', choices=sorted(ALL_SCENARIOS))
    parser.add_argument('--output-dir', default='')
    args = parser.parse_args(argv)
    output_dir = args.output_dir or default_output_dir()
    write_static_artifacts(output_dir)
    result = run_scenario(args.scenario, output_dir)
    write_scenario_result(output_dir, result)
    _write_suite_summary(output_dir, [result])
    print('[SIM] {} -> {} expected={} pass={}'.format(result.name, result.terminal_state, result.expected_terminal, result.passed))
    print('[SIM] output={}'.format(output_dir))
    return 0 if result.passed and not result.nonzero_authority_conflict else 1


if __name__ == '__main__':
    raise SystemExit(main())
