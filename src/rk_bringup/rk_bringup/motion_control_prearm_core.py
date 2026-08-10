"""正式 motion-control prearm 的纯只读决策规则。

此模块不导入 ROS 或 SDK。经典静止已观测到 ``name=mcf``，所以 prearm
只记录状态与 responder，不得由模糊 mode/status 推导出任何写入恢复动作。
"""


def mode_recovery_plan(result, form, name, recovery_enabled=False):
    """保留旧函数名以兼容测试；返回值明确表示 prearm 不可变更状态。"""
    del recovery_enabled
    if result != 0 or form != '0':
        return 'MOTION_SWITCHER_CHECK_FAILED'
    return 'MCF_OBSERVED_NO_MUTATION' if name == 'mcf' else 'MODE_OBSERVED_NO_MUTATION'


def sport_recovery_plan(server_version, recovery_enabled=False):
    """版本 RPC 是 responder 证据；空值 fail-closed，禁止服务开关。"""
    del recovery_enabled
    if server_version:
        return 'SPORT_RPC_READY_NO_MUTATION'
    return 'SPORT_RPC_UNAVAILABLE_NO_MUTATION'


def startup_stop_plan(stop_result):
    """prearm 通过后 startup StopMove 仍只允许一次，非零立即失败。"""
    return 'READY' if stop_result == 0 else 'STARTUP_STOP_FAILED_NO_RETRY'
