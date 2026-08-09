#!/usr/bin/env python3
"""RUN 1 专用单次巡线 adapter/recorder；唯一非零出口为 /control/line_cmd。

此工具不属于 production launch。它只在已经通过静态门且操作者明确授权后运行：
收到新鲜、可见且稳定的 follower candidate 才解除 mux estop，持续至 1.0 秒
立即归零；1.2 秒独立 watchdog 与所有感知/SDK 异常均 fail-closed。
"""

import argparse
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rk_interfaces.msg import LineTrack, SpecialTargetDetection
from rk_bringup.validation_dynamic_core import white_bar_metrics


def _finite(value):
    return isinstance(value, (float, int)) and math.isfinite(float(value))


class SingleDynamicRun(Node):
    """把 follower 原始 steering 受限转发一次，并完整保存 SDK/感知证据。"""

    def __init__(self, args):
        super().__init__('validation_dynamic_run_once')
        self.args = args
        self.publisher = self.create_publisher(Twist, '/control/line_cmd', 10)
        self.estop = self.create_client(SetBool, '/safety/estop')
        self.latest_candidate = None
        self.latest_line = None
        self.latest_white = None
        self.statuses = []
        self.lines = []
        self.whites = []
        self.yaws = []
        self.stop_reason = None
        self.t0_ns = None
        self.tstop_ns = None
        self.first_nonzero_ns = None
        self.arm_ns = None
        self.estop_future = None
        self.estop_target = None
        self.zero_sent = 0
        self.record_file = Path(args.record_path).open('w', encoding='utf-8')
        self.timer = self.create_timer(0.02, self._tick)
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Twist, '/navigation/line_follow_cmd_suggested',
                                 self._on_candidate, 10)
        self.create_subscription(LineTrack, '/perception/line_track',
                                 self._on_line, 10)
        self.create_subscription(SpecialTargetDetection,
                                 '/perception/white_bar_detection',
                                 self._on_white, 10)
        self.create_subscription(String, '/go2/sdk_motion_status',
                                 self._on_status, status_qos)
        self.create_subscription(Twist, '/navigation/cmd_vel',
                                 self._on_final_cmd, 10)
        self.create_subscription(String, '/control/cmd_mux_status',
                                 self._on_mux_status, 10)

    @staticmethod
    def _stamp():
        return time.monotonic_ns()

    def _on_candidate(self, message):
        now = self._stamp()
        self.latest_candidate = (now, message)
        self._record('suggested', now, self._twist_record(message))

    def _on_line(self, message):
        now = self._stamp()
        self.latest_line = (now, message)
        self.lines.append({'monotonic_ns': now, 'visible': bool(message.line_visible),
                           'lateral': float(message.lateral_error),
                           'heading': float(message.heading_error),
                           'confidence': float(message.confidence),
                           'header_sec': int(message.header.stamp.sec),
                           'header_nanosec': int(message.header.stamp.nanosec),
                           'frame_id': str(message.header.frame_id)})
        self._record('line_track', now, self.lines[-1])

    def _on_white(self, message):
        now = self._stamp()
        self.latest_white = (now, message)
        self.whites.append({'monotonic_ns': now, 'visible': bool(message.visible),
                            'confidence': float(message.confidence),
                            'center_x': float(message.center_x),
                            'center_y': float(message.center_y),
                            'area_ratio': float(message.area_ratio),
                            'width_ratio': float(message.width_ratio),
                            'height_ratio': float(message.height_ratio),
                            'inside_candidate': bool(message.inside_candidate),
                            'direction_hint': str(message.direction_hint),
                            'reason': str(message.reason),
                            'header_sec': int(message.header.stamp.sec),
                            'header_nanosec': int(message.header.stamp.nanosec),
                            'frame_id': str(message.header.frame_id)})
        self._record('white_bar', now, self.whites[-1])

    @staticmethod
    def _twist_record(message):
        return {'linear_x': float(message.linear.x), 'linear_y': float(message.linear.y),
                'angular_z': float(message.angular.z)}

    def _record(self, channel, now, payload):
        """每条链路数据立即 JSONL 落盘，异常退出也保留第一条 zero 证据。"""
        self.record_file.write(json.dumps(
            {'channel': channel, 'monotonic_ns': now, 'data': payload},
            separators=(',', ':'), allow_nan=False,
        ) + '\n')
        self.record_file.flush()

    def _on_final_cmd(self, message):
        self._record('final_cmd', self._stamp(), self._twist_record(message))

    def _on_mux_status(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            payload = {'raw': message.data}
        self._record('mux_status', self._stamp(), payload)
        # 一旦真实 MOVE 已获 ACK，mux 必须持续选择 line；mission 或任何
        # 其它 source 都是本轮隔离合同被破坏，立即归零并重新 estop。
        if (
            self.t0_ns is not None
            and self.stop_reason is None
            and payload.get('active_source') != 'line'
        ):
            self._stop('MUX_SOURCE_SWITCH_{}'.format(
                payload.get('active_source', 'unknown')
            ))

    def _on_status(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        # T0 只能来自本轮正式启动产生的 server instance；transient-local
        # 旧状态绝不能把本次非零命令误判为已被 SDK 接受。
        if payload.get('server_instance_id') != self.args.server_instance_id:
            return
        payload['receive_monotonic_ns'] = self._stamp()
        self.statuses.append(payload)
        self._record('sdk_status', payload['receive_monotonic_ns'], payload)
        if payload.get('event') == 'SDK_ERROR':
            self._stop('SDK_ERROR')
        if payload.get('event') == 'MOVE' and payload.get('ret') != 0:
            self._stop('MOVE_NOT_ACCEPTED')
        if self.t0_ns is None and payload.get('event') == 'MOVE' and payload.get('ret') == 0:
            self.t0_ns = payload['receive_monotonic_ns']

    def _request_estop(self, enabled):
        """服务不可用或服务失败均保持/恢复 estop，绝不继续发非零命令。"""
        if self.estop_future is not None:
            return
        if not self.estop.wait_for_service(timeout_sec=0.0):
            self._stop('ESTOP_SERVICE_UNAVAILABLE')
            return
        request = SetBool.Request()
        request.data = bool(enabled)
        self.estop_target = bool(enabled)
        self.estop_future = self.estop.call_async(request)

    def _poll_estop(self):
        if self.estop_future is None or not self.estop_future.done():
            return None
        future, self.estop_future = self.estop_future, None
        try:
            response = future.result()
        except Exception:
            self._stop('ESTOP_SERVICE_ERROR')
            return False
        if response is None or not response.success:
            self._stop('ESTOP_SERVICE_REJECTED')
            return False
        return self.estop_target

    def _line_safe(self, now):
        if self.latest_line is None:
            return False
        stamp, line = self.latest_line
        return (
            now - stamp <= int(self.args.line_timeout_sec * 1e9)
            and bool(line.line_visible)
            and _finite(line.lateral_error)
            and abs(float(line.lateral_error)) <= self.args.max_lateral_error
            and _finite(line.heading_error)
        )

    def _candidate_safe(self, now):
        if self.latest_candidate is None:
            return False
        stamp, command = self.latest_candidate
        return (
            now - stamp <= int(self.args.candidate_timeout_sec * 1e9)
            and _finite(command.linear.x) and float(command.linear.x) > 0.0
            and _finite(command.angular.z)
            and abs(float(command.angular.z)) <= self.args.max_yaw
        )

    def _publish_zero(self):
        self.publisher.publish(Twist())
        self._record('adapter_line_cmd', self._stamp(), {
            'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0,
            'reason': self.stop_reason or 'explicit_zero',
        })
        self.zero_sent += 1

    def _stop(self, reason):
        if self.stop_reason is None:
            self.stop_reason = reason
            self.tstop_ns = self._stamp()
        self._publish_zero()
        self._request_estop(True)

    def _tick(self):
        now = self._stamp()
        estop_change = self._poll_estop()
        if self.stop_reason is not None:
            self._publish_zero()
            return
        # 先积累 0.5 秒的稳定静态证据，再解除 estop；未达到时永远为零。
        if self.arm_ns is None:
            if self._line_safe(now) and self._candidate_safe(now):
                self.arm_ns = now
            else:
                # mux estop 已保证 final zero；arm 前向 line topic 写零会在
                # ROS 队列滞留，解除 estop 后反而可能覆盖首条非零命令。
                return
        if self.t0_ns is None:
            if now - self.arm_ns < 500_000_000:
                return
            if self.estop_target is not False and self.estop_future is None:
                self._request_estop(False)
                return
            if estop_change is not False and self.estop_target is False:
                # 只有服务明确成功后才发布第一条非零 command。
                if self.estop_future is not None:
                    return
                if not self._line_safe(now) or not self._candidate_safe(now):
                    self._stop('LINE_TRACK_FAILURE_BEFORE_MOVE')
                    return
        # SDK ACK 未到达也不能让第一条非零 command 无界持续；以 ACK 为
        # 首选 T0，缺失时退回首条 command 的保守时钟执行同样的上限。
        motion_reference_ns = self.t0_ns or self.first_nonzero_ns
        if motion_reference_ns is not None and now - motion_reference_ns >= int(self.args.motion_sec * 1e9):
            self._stop('DURATION_LIMIT')
            return
        if motion_reference_ns is not None and now - motion_reference_ns > int(self.args.watchdog_sec * 1e9):
            self._stop('WATCHDOG_LIMIT')
            return
        if not self._line_safe(now):
            self._stop('LINE_TRACK_FAILURE')
            return
        if not self._candidate_safe(now):
            self._stop('CANDIDATE_STALE')
            return
        command = self.latest_candidate[1]
        output = Twist()
        output.linear.x = min(float(command.linear.x), self.args.max_vx)
        output.linear.y = 0.0
        output.angular.z = float(command.angular.z)
        self.yaws.append(output.angular.z)
        if self.first_nonzero_ns is None:
            self.first_nonzero_ns = now
        self.publisher.publish(output)
        self._record('adapter_line_cmd', now, {
            'linear_x': output.linear.x, 'linear_y': output.linear.y,
            'angular_z': output.angular.z, 'reason': 'armed_forward',
        })

    def finished(self):
        return self.stop_reason is not None and self.zero_sent >= 3 and self.estop_future is None

    def report(self):
        def values(key, predicate=lambda row: True):
            return [row[key] for row in self.lines if predicate(row) and _finite(row[key])]
        lateral = values('lateral')
        heading = values('heading')
        moves = [row for row in self.statuses if row.get('event') == 'MOVE']
        stops = [row for row in self.statuses if row.get('event') == 'STOP_MOVE']
        errors = [row for row in self.statuses if row.get('event') == 'SDK_ERROR']
        return {
            'stop_reason': self.stop_reason, 't0_ns': self.t0_ns,
            'tstop_ns': self.tstop_ns,
            'motion_duration_sec': None if not self.t0_ns or not self.tstop_ns else (self.tstop_ns-self.t0_ns)/1e9,
            'sdk': {'move_count': len(moves), 'move_ret': [x.get('ret') for x in moves],
                    'stop_move_ret': [x.get('ret') for x in stops], 'sdk_error_count': len(errors),
                    'statuses': self.statuses},
            'line': {'samples': len(self.lines), 'valid_samples': sum(x['visible'] for x in self.lines),
                     'lateral_mean': sum(lateral)/len(lateral) if lateral else None,
                     'lateral_std': (sum((x-sum(lateral)/len(lateral))**2 for x in lateral)/len(lateral))**0.5 if lateral else None,
                     'lateral_max_abs': max(map(abs,lateral)) if lateral else None,
                     'heading_mean': sum(heading)/len(heading) if heading else None,
                     'yaw_mean': sum(self.yaws)/len(self.yaws) if self.yaws else None,
                     'yaw_max_abs': max(map(abs,self.yaws)) if self.yaws else None,
                     'loss_frames': sum(not x['visible'] for x in self.lines)},
            'white_bar': white_bar_metrics(self.whites, self.t0_ns, self.tstop_ns),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion-sec', type=float, default=1.0)
    parser.add_argument('--watchdog-sec', type=float, default=1.2)
    parser.add_argument('--max-vx', type=float, default=0.25)
    parser.add_argument('--max-yaw', type=float, default=0.8)
    parser.add_argument('--max-lateral-error', type=float, default=0.8)
    parser.add_argument('--line-timeout-sec', type=float, default=0.35)
    parser.add_argument('--candidate-timeout-sec', type=float, default=0.25)
    parser.add_argument('--server-instance-id', required=True)
    parser.add_argument('--record-path', required=True)
    args = parser.parse_args()
    if not (0 < args.motion_sec <= 1.0 and args.motion_sec < args.watchdog_sec <= 1.2):
        parser.error('motion/watchdog limits must satisfy 0 < motion <= 1.0 < watchdog <= 1.2')
    rclpy.init()
    node = SingleDynamicRun(args)
    deadline = time.monotonic() + 20.0
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.finished():
            rclpy.spin_once(node, timeout_sec=0.02)
        if node.stop_reason is None:
            node._stop('ARM_TIMEOUT')
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.02)
        print('VALIDATION_DYNAMIC_RUN ' + json.dumps(node.report(), separators=(',', ':'), allow_nan=False))
    finally:
        node._stop(node.stop_reason or 'PROCESS_EXIT')
        node.record_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
