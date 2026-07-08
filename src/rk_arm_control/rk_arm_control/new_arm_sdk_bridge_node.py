#!/usr/bin/env python3

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String


class BridgeCommandError(RuntimeError):
    """桥接命令格式错误或当前 SDK 未接好。"""


class NewArmSdkBridgeNode(Node):
    """新机械臂 SDK 桥接节点。

    上层 `new_arm_task_node` 只负责比赛任务流程，并把动作发布到
    `/arm/sdk_bridge/command_json`。本节点负责订阅该 topic，然后把通用
    JSON 转成厂家 SDK 调用。

    当前模板支持两种模式：
    - mock：只校验、打印和发布状态，不控制真实硬件；
    - real：预留真实 SDK 接入点，默认会拒绝执行，避免误动机械臂。
    """

    def __init__(self) -> None:
        super().__init__('new_arm_sdk_bridge_node')

        self.declare_parameter('config_file', '')
        self.declare_parameter('bridge_mode', '')

        config_file = str(self.get_parameter('config_file').value or '')
        config = self._load_config(config_file)

        new_arm_config = dict(config.get('new_arm', {}))
        adapter_config = dict(new_arm_config.get('adapter', {}))
        sdk_adapter_config = dict(adapter_config.get('sdk_bridge', {}))
        bridge_config = dict(new_arm_config.get('sdk_bridge_node', {}))

        self._command_topic = str(
            bridge_config.get(
                'command_topic',
                sdk_adapter_config.get(
                    'command_topic',
                    '/arm/sdk_bridge/command_json',
                ),
            )
        )
        self._status_topic = str(
            bridge_config.get('status_topic', '/arm/sdk_bridge/status')
        )

        mode_from_param = str(self.get_parameter('bridge_mode').value or '')
        self._bridge_mode = (
            mode_from_param
            or str(bridge_config.get('bridge_mode', 'mock'))
        ).strip().lower()

        self._expected_joint_count = int(
            bridge_config.get('expected_joint_count', 6)
        )
        self._strict_joint_count = bool(
            bridge_config.get('strict_joint_count', False)
        )
        self._min_duration_sec = float(
            bridge_config.get('min_duration_sec', 0.05)
        )
        self._max_duration_sec = float(
            bridge_config.get('max_duration_sec', 10.0)
        )
        self._simulate_duration_wait = bool(
            bridge_config.get('simulate_duration_wait', False)
        )
        self._real_config = dict(bridge_config.get('real', {}))
        self._real_sdk_enabled = bool(
            self._real_config.get('enable_real_sdk', False)
        )

        self._status_pub = self.create_publisher(String, self._status_topic, 10)
        self._command_sub = self.create_subscription(
            String,
            self._command_topic,
            self._on_command,
            10,
        )

        self.get_logger().info(
            'new arm SDK bridge ready: '
            f'mode={self._bridge_mode}, '
            f'command_topic={self._command_topic}, '
            f'status_topic={self._status_topic}'
        )
        if self._bridge_mode == 'real' and not self._real_sdk_enabled:
            self.get_logger().warn(
                'bridge_mode=real 但 real.enable_real_sdk=false，'
                '收到真实动作时会拒绝执行。'
            )

    def _load_config(self, config_file: str) -> Dict[str, Any]:
        if not config_file:
            self.get_logger().warn(
                '未设置 config_file，使用 SDK bridge 默认参数。'
            )
            return {}

        path = Path(config_file).expanduser()
        if not path.exists():
            self.get_logger().warn(
                f'配置文件不存在：{path}，使用 SDK bridge 默认参数。'
            )
            return {}

        with path.open('r', encoding='utf-8') as file_obj:
            data = yaml.safe_load(file_obj) or {}
        if not isinstance(data, dict):
            self.get_logger().warn(
                f'配置文件格式不是 YAML dict：{path}，使用默认参数。'
            )
            return {}
        self.get_logger().info(f'loaded SDK bridge config: {path}')
        return data

    def _on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise BridgeCommandError('JSON 根节点必须是 object/dict')
            command = self._normalize_command(payload)
            self._publish_status(
                payload,
                command,
                'RECEIVED',
                True,
                'command received',
            )
            self._dispatch(command, payload)
        except Exception as exc:  # noqa: BLE001 - ROS 回调要吞掉异常并上报状态
            self.get_logger().error(f'SDK bridge command failed: {exc}')
            self._publish_status(
                self._safe_payload(msg.data),
                'UNKNOWN',
                'FAILED',
                False,
                str(exc),
            )

    def _normalize_command(self, payload: Dict[str, Any]) -> str:
        command = str(payload.get('command', '')).strip().upper()
        if command:
            return command

        # 兼容 D1 SDK 示例风格的参考 JSON，便于对照调试。
        # 新机械臂正式接入时仍建议使用 generic_json。
        funcode = payload.get('funcode')
        if funcode == 2:
            return 'MOVE_JOINTS_D1_REFERENCE'
        if funcode == 1:
            return 'MOVE_SINGLE_JOINT_D1_REFERENCE'
        raise BridgeCommandError('缺少 command 字段')

    def _dispatch(self, command: str, payload: Dict[str, Any]) -> None:
        if command == 'MOVE_JOINTS':
            joints = self._read_joints(payload)
            duration_sec = self._read_duration(payload)
            pose_name = str(payload.get('pose_name', ''))
            self._handle_move_joints(payload, joints, duration_sec, pose_name)
            return

        if command == 'GRIPPER':
            action = str(payload.get('action', '')).strip().lower()
            duration_sec = self._read_duration(payload)
            self._handle_gripper(payload, action, duration_sec)
            return

        if command == 'STOP':
            self._handle_stop(payload)
            return

        if command == 'MOVE_JOINTS_D1_REFERENCE':
            joints = self._read_d1_reference_joints(payload)
            duration_sec = self._read_duration(payload)
            pose_name = str(payload.get('pose_name', 'd1_reference'))
            self._handle_move_joints(payload, joints, duration_sec, pose_name)
            return

        raise BridgeCommandError(f'暂不支持的 command：{command}')

    def _read_joints(self, payload: Dict[str, Any]) -> List[float]:
        raw_joints = payload.get('joints')
        if not isinstance(raw_joints, list):
            raise BridgeCommandError('MOVE_JOINTS 需要 joints 数组')

        joints = [float(value) for value in raw_joints]
        if not joints:
            raise BridgeCommandError('joints 不能为空')

        if len(joints) != self._expected_joint_count:
            message = (
                f'joints 数量={len(joints)}，'
                f'期望={self._expected_joint_count}'
            )
            if self._strict_joint_count:
                raise BridgeCommandError(message)
            self.get_logger().warn(message)
        return joints

    def _read_d1_reference_joints(self, payload: Dict[str, Any]) -> List[float]:
        data = payload.get('data', {})
        if not isinstance(data, dict):
            raise BridgeCommandError('D1 reference payload 缺少 data dict')

        joints = []
        for index in range(self._expected_joint_count):
            key = f'angle{index}'
            if key not in data:
                break
            joints.append(float(data[key]))

        if not joints:
            raise BridgeCommandError('D1 reference payload 未找到 angle0..')
        return joints

    def _read_duration(self, payload: Dict[str, Any]) -> float:
        duration_sec = float(payload.get('duration_sec', 0.5))
        if duration_sec < self._min_duration_sec:
            raise BridgeCommandError(
                f'duration_sec={duration_sec:.3f} 小于最小值 '
                f'{self._min_duration_sec:.3f}'
            )
        if duration_sec > self._max_duration_sec:
            raise BridgeCommandError(
                f'duration_sec={duration_sec:.3f} 大于最大值 '
                f'{self._max_duration_sec:.3f}'
            )
        return duration_sec

    def _handle_move_joints(
        self,
        payload: Dict[str, Any],
        joints: List[float],
        duration_sec: float,
        pose_name: str,
    ) -> None:
        if self._bridge_mode == 'mock':
            self.get_logger().info(
                '[MOCK SDK] move_joints '
                f'pose={pose_name} joints={joints} '
                f'duration_sec={duration_sec:.2f}'
            )
            self._maybe_wait(duration_sec)
        elif self._bridge_mode == 'real':
            self._real_move_joints(joints, duration_sec, pose_name)
        else:
            raise BridgeCommandError(
                f'未知 bridge_mode={self._bridge_mode}，请使用 mock 或 real'
            )

        self._publish_status(
            payload,
            'MOVE_JOINTS',
            'DONE',
            True,
            f'move_joints completed: {pose_name}',
        )

    def _handle_gripper(
        self,
        payload: Dict[str, Any],
        action: str,
        duration_sec: float,
    ) -> None:
        if action not in ('open', 'close'):
            raise BridgeCommandError(f'夹爪 action 只能是 open/close：{action}')

        if self._bridge_mode == 'mock':
            self.get_logger().info(
                f'[MOCK SDK] gripper action={action} '
                f'duration_sec={duration_sec:.2f}'
            )
            self._maybe_wait(duration_sec)
        elif self._bridge_mode == 'real':
            self._real_gripper(action, duration_sec)
        else:
            raise BridgeCommandError(
                f'未知 bridge_mode={self._bridge_mode}，请使用 mock 或 real'
            )

        self._publish_status(
            payload,
            'GRIPPER',
            'DONE',
            True,
            f'gripper {action} completed',
        )

    def _handle_stop(self, payload: Dict[str, Any]) -> None:
        if self._bridge_mode == 'mock':
            self.get_logger().warn('[MOCK SDK] stop requested')
        elif self._bridge_mode == 'real':
            self._real_stop()
        else:
            raise BridgeCommandError(
                f'未知 bridge_mode={self._bridge_mode}，请使用 mock 或 real'
            )

        self._publish_status(
            payload,
            'STOP',
            'DONE',
            True,
            'stop completed',
        )

    def _maybe_wait(self, duration_sec: float) -> None:
        if not self._simulate_duration_wait:
            return
        time.sleep(max(0.0, duration_sec))

    def _real_move_joints(
        self,
        joints: List[float],
        duration_sec: float,
        pose_name: str,
    ) -> None:
        self._require_real_sdk()

        # TODO：新机械臂 SDK 到货后，在这里替换成厂家函数。
        # 示例伪代码：
        # self._sdk.move_joints(joints, duration=duration_sec)
        # self._sdk.wait_motion_done(timeout=duration_sec + 1.0)
        raise BridgeCommandError(
            'real move_joints 尚未接入厂家 SDK，'
            f'pose={pose_name}, joints={joints}'
        )

    def _real_gripper(self, action: str, duration_sec: float) -> None:
        self._require_real_sdk()

        # TODO：新机械臂 SDK 到货后，在这里替换成夹爪开合函数。
        # 示例伪代码：
        # if action == 'open':
        #     self._sdk.open_gripper()
        # else:
        #     self._sdk.close_gripper()
        raise BridgeCommandError(
            'real gripper 尚未接入厂家 SDK，'
            f'action={action}, duration_sec={duration_sec:.2f}'
        )

    def _real_stop(self) -> None:
        self._require_real_sdk()

        # TODO：新机械臂 SDK 到货后，在这里替换成急停/停止函数。
        raise BridgeCommandError('real stop 尚未接入厂家 SDK')

    def _require_real_sdk(self) -> None:
        if not self._real_sdk_enabled:
            raise BridgeCommandError(
                'real.enable_real_sdk=false，已阻止真实机械臂动作。'
            )

    def _publish_status(
        self,
        payload: Any,
        command: str,
        state: str,
        success: bool,
        message: str,
    ) -> None:
        seq = None
        if isinstance(payload, dict):
            seq = payload.get('seq')

        status = {
            'seq': seq,
            'command': command,
            'state': state,
            'success': success,
            'message': message,
            'bridge_mode': self._bridge_mode,
            'stamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False, separators=(',', ':'))
        self._status_pub.publish(msg)

    def _safe_payload(self, data: str) -> Dict[str, Any]:
        try:
            payload = json.loads(data)
            if isinstance(payload, dict):
                return payload
        except Exception:  # noqa: BLE001 - 只用于错误状态兜底
            pass
        return {}


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = NewArmSdkBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
