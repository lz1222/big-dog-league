"""正式 motion-control prearm 的纯决策规则。

此模块不导入 ROS 或 SDK；仅把一次性恢复允许条件显式化，供回归测试确保
运行中不会因 mode/status 的模糊值触发额外 ReleaseMode 或 ServiceSwitch。
"""


def mode_recovery_plan(result, form, name, recovery_enabled):
    """根据原始 CheckMode 结果返回唯一允许的启动前动作。"""
    if result != 0 or form != '0':
        return 'MOTION_SWITCHER_CHECK_FAILED'
    if name == '':
        return 'NO_RELEASE'
    if name == 'mcf':
        return 'RELEASE_ONCE' if recovery_enabled else 'RECOVERY_DISABLED'
    return 'UNKNOWN_MOTION_MODE'


def sport_recovery_plan(server_version, recovery_enabled):
    """版本 RPC 是 responder 证据；ServiceList status 不参与 readiness 判断。"""
    if server_version:
        return 'NO_SERVICE_SWITCH'
    return 'SWITCH_OFF_ON_ONCE' if recovery_enabled else 'SPORT_RPC_RECOVERY_DISABLED'


def startup_stop_plan(stop_result):
    """prearm 通过后 startup StopMove 仍只允许一次，非零立即失败。"""
    return 'READY' if stop_result == 0 else 'STARTUP_STOP_FAILED_NO_RETRY'
