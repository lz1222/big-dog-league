#!/usr/bin/env python3
"""V2 UDP Forwarder — session/seq追踪 + 高频持续发布

UDP包格式:  version session_id seq monotonic_ns vx vy wz flags
"""

import math, socket, struct, time, threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


@dataclass
class ForwarderConfig:
    cmd_vel_topic: str = '/navigation/cmd_vel'
    udp_host: str = '127.0.0.1'
    udp_port: int = 15001
    max_vx: float = 0.60
    max_vy: float = 0.05
    max_yaw: float = 0.60
    deadband: float = 0.01
    timeout_sec: float = 0.30
    publish_rate_hz: float = 50.0  # high frequency to prevent SDK watchdog
    protocol_version: int = 1      # 1=v1 "vx vy wz", 2=v2 with session/seq


class V2Forwarder(Node):
    def __init__(self):
        super().__init__('v2_forwarder')

        # Config
        self.cfg = ForwarderConfig(
            cmd_vel_topic=self.declare_parameter('cmd_vel_topic', '/navigation/cmd_vel').value,
            udp_host=self.declare_parameter('udp_host', '127.0.0.1').value,
            udp_port=self.declare_parameter('udp_port', 15001).value,
            max_vx=self.declare_parameter('max_vx', 0.60).value,
            max_vy=self.declare_parameter('max_vy', 0.05).value,
            max_yaw=self.declare_parameter('max_yaw', 0.60).value,
            deadband=self.declare_parameter('deadband', 0.01).value,
            timeout_sec=self.declare_parameter('timeout_sec', 0.30).value,
            publish_rate_hz=self.declare_parameter('publish_rate_hz', 50.0).value,
            protocol_version=self.declare_parameter('protocol_version', 1).value,
        )

        # State
        self._lock = threading.Lock()
        self._session_id = int(time.monotonic() * 1e9) & 0xFFFFFFFFFFFFFFFF
        self._seq: int = 0
        self._last_cmd_time: float = 0.0
        self._current_vx: float = 0.0
        self._current_vy: float = 0.0
        self._current_wz: float = 0.0
        self._flags: int = 0
        self._sock: Optional[socket.socket] = None
        self._sent_count: int = 0

        # UDP socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # ROS subscription
        self.create_subscription(Twist, self.cfg.cmd_vel_topic, self._on_twist, 10)

        # High-frequency publish timer
        period = 1.0 / max(1.0, self.cfg.publish_rate_hz)
        self.create_timer(period, self._publish_loop)

        # Watchdog timer (check staleness)
        self.create_timer(0.05, self._watchdog_check)

        self.get_logger().info(
            f'V2 Forwarder started: {self.cfg.cmd_vel_topic} -> '
            f'{self.cfg.udp_host}:{self.cfg.udp_port} '
            f'session={self._session_id} '
            f'rate={self.cfg.publish_rate_hz}Hz '
            f'limits=({self.cfg.max_vx},{self.cfg.max_vy},{self.cfg.max_yaw}) '
            f'deadband={self.cfg.deadband} timeout={self.cfg.timeout_sec}s'
        )

    def _on_twist(self, msg: Twist):
        """Receive ROS Twist, update latest command."""
        with self._lock:
            vx = float(msg.linear.x)
            vy = float(msg.linear.y)
            wz = float(msg.angular.z)

            # Validate
            if not all(math.isfinite(v) for v in (vx, vy, wz)):
                self.get_logger().error(f'Non-finite Twist: ({vx},{vy},{wz}) → STOP')
                self._current_vx = self._current_vy = self._current_wz = 0.0
                self._flags = 0x02  # emergency stop
                self._last_cmd_time = time.monotonic()
                return

            # Clamp
            vx = max(-self.cfg.max_vx, min(self.cfg.max_vx, vx))
            vy = max(-self.cfg.max_vy, min(self.cfg.max_vy, vy))
            wz = max(-self.cfg.max_yaw, min(self.cfg.max_yaw, wz))

            # Deadband
            if abs(vx) <= self.cfg.deadband: vx = 0.0
            if abs(vy) <= self.cfg.deadband: vy = 0.0
            if abs(wz) <= self.cfg.deadband: wz = 0.0

            self._current_vx = vx
            self._current_vy = vy
            self._current_wz = wz
            self._flags = 0  # clear emergency
            self._last_cmd_time = time.monotonic()

    def _publish_loop(self):
        """High-frequency UDP publish (50Hz)."""
        with self._lock:
            vx, vy, wz = self._current_vx, self._current_vy, self._current_wz
            flags = self._flags
            seq = self._seq
            session = self._session_id

        self._seq += 1
        mono_ns = int(time.monotonic() * 1e9) & 0x7FFFFFFFFFFFFFFF

        # Format: v1 = "vx vy wz", v2 = "2 session seq mono_ns vx vy wz flags"
        if self.cfg.protocol_version >= 2:
            packet = f"2 {session} {seq} {mono_ns} {vx:.4f} {vy:.4f} {wz:.4f} {flags}"
        else:
            packet = f"{vx:.4f} {vy:.4f} {wz:.4f}"

        try:
            self._sock.sendto(packet.encode(), (self.cfg.udp_host, self.cfg.udp_port))
            self._sent_count += 1
        except OSError as e:
            self.get_logger().error(f'UDP send failed: {e}')

    def _watchdog_check(self):
        """Check if upstream commands are stale."""
        with self._lock:
            age = time.monotonic() - self._last_cmd_time
            if age >= self.cfg.timeout_sec and self._last_cmd_time > 0:
                self._current_vx = self._current_vy = self._current_wz = 0.0
                if self._sent_count % 100 == 0:
                    self.get_logger().warn(
                        f'WATCHDOG_CMD_AGE: {age:.3f}s > {self.cfg.timeout_sec}s → zero'
                    )


def main():
    rclpy.init()
    node = V2Forwarder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
