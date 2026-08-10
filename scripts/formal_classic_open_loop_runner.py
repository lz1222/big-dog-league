#!/usr/bin/env python3
"""经典步态开环直线的受管候选源。

本工具不创建第二条控制链：它只向正式 mux 已有的 locomotion 候选 topic
发布固定 Twist，最终 /navigation/cmd_vel 仍只能由 command_mux_node 发布。
计时以 mux 后最终命令为准；任何非零横移、转向、SDK Move 错误或源异常都会
停止候选并请求既有 estop 服务，使 mux 立即恢复零输出。
"""

import argparse
import json
import math
import time
from pathlib import Path

from geometry_msgs.msg import Twist
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class FormalClassicOpenLoopRunner(Node):
    """通过既有 locomotion 候选源执行一次有界的纯前向开环窗口。"""

    def __init__(self, arguments):
        super().__init__('formal_classic_open_loop_runner')
        self.args = arguments
        self.publisher = self.create_publisher(Twist, arguments.command_topic, 10)
        self.estop = self.create_client(SetBool, arguments.estop_service)
        self.create_subscription(Twist, arguments.final_topic, self._on_final, 20)
        self.create_subscription(String, arguments.mux_topic, self._on_mux, 20)
        self.create_subscription(String, arguments.sdk_topic, self._on_sdk, 100)
        self.create_subscription(Odometry, arguments.odom_topic, self._on_odom, 50)
        self.phase = 'PRE'
        self.started_ns = time.monotonic_ns()
        self.last_final = None
        self.last_mux = None
        self.last_sdk = None
        self.last_odom = None
        self.active_ns = 0
        self.last_active_sample_ns = None
        self.run_started = False
        self.abort_reason = None
        self.events = []
        self.stop_requested = False
        self.done = False
        self.timer = self.create_timer(0.02, self._tick)

    def _event(self, name, **data):
        self.events.append({'monotonic_ns': time.monotonic_ns(), 'event': name, **data})

    def _on_final(self, message):
        self.last_final = (time.monotonic_ns(), float(message.linear.x),
                           float(message.linear.y), float(message.angular.z))

    def _on_mux(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self._abort('INVALID_MUX_STATUS')
            return
        self.last_mux = (time.monotonic_ns(), payload)

    def _on_sdk(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.last_sdk = (time.monotonic_ns(), payload)
        # 只把当前实例的 MOVE 非零返回视为执行链故障；其它只读状态均记录。
        if (payload.get('server_instance_id') == self.args.server_instance_id
                and payload.get('event') == 'MOVE'
                and payload.get('ret') != 0):
            self._abort('SDK_MOVE_ERROR_{}'.format(payload.get('ret')))

    def _on_odom(self, message):
        self.last_odom = time.monotonic_ns()

    def _publish_candidate(self):
        command = Twist()
        command.linear.x = self.args.vx
        command.linear.y = 0.0
        command.angular.z = 0.0
        self.publisher.publish(command)

    def _publish_zero(self):
        self.publisher.publish(Twist())

    def _request_estop(self):
        if self.stop_requested:
            return
        self.stop_requested = True
        if self.estop.wait_for_service(timeout_sec=0.2):
            request = SetBool.Request()
            request.data = True
            self.estop.call_async(request)
        else:
            self._event('ESTOP_SERVICE_UNAVAILABLE')

    def _abort(self, reason):
        if self.abort_reason is None:
            self.abort_reason = reason
            self._event('ABORT', reason=reason)

    def _fresh(self, sample, now_ns, timeout_sec=0.4):
        return sample is not None and now_ns - sample[0] <= int(timeout_sec * 1e9)

    def _zero_observation_valid(self, now_ns):
        if not self._fresh(self.last_final, now_ns) or not self._fresh(self.last_mux, now_ns):
            return False
        _, vx, vy, wz = self.last_final
        status = self.last_mux[1]
        return (abs(vx) <= self.args.epsilon and abs(vy) <= self.args.epsilon
                and abs(wz) <= self.args.epsilon
                and status.get('active_source') in ('none', 'estop')
                and not bool(status.get('estop')))

    def _active_now(self, now_ns):
        if not self._fresh(self.last_final, now_ns) or not self._fresh(self.last_mux, now_ns):
            return False
        _, vx, vy, wz = self.last_final
        status = self.last_mux[1]
        if abs(vy) > self.args.epsilon:
            self._abort('UNEXPECTED_FINAL_VY')
        if abs(wz) > self.args.epsilon:
            self._abort('UNEXPECTED_FINAL_WZ')
        if status.get('active_source') != 'locomotion':
            return False
        return vx > 0.0 and abs(vy) <= self.args.epsilon and abs(wz) <= self.args.epsilon

    def _finish(self, result):
        if self.phase == 'DONE':
            return
        self._publish_zero()
        self._request_estop()
        self.phase = 'POST'
        self.post_started_ns = time.monotonic_ns()
        self._event('MOTION_END', result=result, active_sec=self.active_ns / 1e9)

    def _tick(self):
        now_ns = time.monotonic_ns()
        if self.phase == 'PRE':
            # 外部正式链已完成零输出确认时可设为 0，避免测试 runner 重复建立门控。
            if self.args.pre_sec == 0.0:
                self.phase = 'MOVE'
                self._event('MOTION_START')
                return
            if now_ns - self.started_ns >= int(self.args.pre_sec * 1e9):
                if self.abort_reason:
                    self._finish('ABORTED')
                elif not self._zero_observation_valid(now_ns):
                    self._abort('FORMAL_ZERO_OBSERVATION_FAILED')
                    self._finish('ABORTED')
                else:
                    self.phase = 'MOVE'
                    self._event('MOTION_START')
            return
        if self.phase == 'MOVE':
            self._publish_candidate()
            active = self._active_now(now_ns)
            if active:
                if self.last_active_sample_ns is not None:
                    self.active_ns += now_ns - self.last_active_sample_ns
                self.last_active_sample_ns = now_ns
                self.run_started = True
            elif self.run_started:
                self._abort('FINAL_COMMAND_OR_SOURCE_INTERRUPTED')
            if self.abort_reason:
                self._finish('ABORTED')
            elif self.active_ns >= int(self.args.active_sec * 1e9):
                self._finish('COMPLETE')
            elif now_ns - self.started_ns > int(self.args.max_wall_sec * 1e9):
                self._abort('ACTIVE_TIME_TIMEOUT')
                self._finish('ABORTED')
            return
        if self.phase == 'POST':
            self._publish_zero()
            if now_ns - self.post_started_ns >= int(self.args.post_sec * 1e9):
                self.phase = 'DONE'
                self._event('POST_COMPLETE')
                self._write_result()
                # 回调内直接 shutdown 在 Foxy 可能阻塞 executor；只标记完成，
                # 由主循环退出后统一发布零并释放 ROS 资源。
                self.done = True
                self.timer.cancel()

    def _write_result(self):
        payload = {
            'result': 'CLASSIC_OPEN_LOOP_7S_COMPLETE' if self.abort_reason is None else 'CLASSIC_OPEN_LOOP_ABORTED_' + self.abort_reason,
            'active_motion_sec': self.active_ns / 1e9,
            'abort_reason': self.abort_reason,
            'command': {'vx': self.args.vx, 'vy': 0.0, 'wz': 0.0},
            'command_topic': self.args.command_topic,
            'final_topic': self.args.final_topic,
            'events': self.events,
        }
        Path(self.args.result_path).write_text(json.dumps(payload, indent=2) + '\n')


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--server-instance-id', required=True)
    parser.add_argument('--result-path', required=True)
    parser.add_argument('--command-topic', default='/control/locomotion_cmd')
    parser.add_argument('--final-topic', default='/navigation/cmd_vel')
    parser.add_argument('--mux-topic', default='/control/cmd_mux_status')
    parser.add_argument('--sdk-topic', default='/go2/sdk_motion_status')
    parser.add_argument('--odom-topic', default='/utlidar/robot_odom')
    parser.add_argument('--estop-service', default='/safety/estop')
    parser.add_argument('--vx', type=float, default=0.27)
    parser.add_argument('--pre-sec', type=float, default=0.0)
    parser.add_argument('--active-sec', type=float, default=7.0)
    parser.add_argument('--post-sec', type=float, default=3.0)
    parser.add_argument('--max-wall-sec', type=float, default=9.0)
    parser.add_argument('--epsilon', type=float, default=1e-3)
    args = parser.parse_args()
    if not (math.isfinite(args.vx) and args.vx == 0.27):
        parser.error('--vx must be exactly 0.27 for this acceptance')
    if not (args.active_sec == 7.0 and args.pre_sec >= 0.0 and args.post_sec >= 3.0):
        parser.error('requires pre>=0, active=7, post>=3')
    return args


def main():
    args = parse_arguments()
    rclpy.init()
    node = FormalClassicOpenLoopRunner(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node._publish_zero()
            node._request_estop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
