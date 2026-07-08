#!/usr/bin/env python3

from typing import List


class ArmHardwareAdapter:
    """机械臂底层适配器基类。

    上层任务节点只调用这个接口，不直接调用某个厂家的 SDK。
    新机械臂到货后，只需要新增或修改一个 Adapter。
    """

    def initialize(self) -> bool:
        """初始化硬件连接。

        DryRun 可以直接返回 True；真实 SDK 可以在这里打开串口、CAN、
        TCP 或 DDS 通道。
        """
        return True

    def move_joints(
        self,
        joints: List[float],
        duration_sec: float,
        pose_name: str = '',
    ) -> bool:
        """移动到一组关节角。

        joints 的单位由 YAML 和具体 Adapter 约定；当前通用框架默认用
        degrees，方便现场照着机械臂调试软件读数填写。
        """
        raise NotImplementedError

    def open_gripper(self, duration_sec: float) -> bool:
        """打开夹爪。"""
        raise NotImplementedError

    def close_gripper(self, duration_sec: float) -> bool:
        """关闭夹爪。"""
        raise NotImplementedError

    def stop(self) -> None:
        """停止当前动作。

        真实硬件必须尽量做成安全停止；如果 SDK 没有急停接口，也要至少
        停止继续发送后续命令。
        """
        raise NotImplementedError

    def shutdown(self) -> None:
        """节点退出时释放资源。"""
        self.stop()

