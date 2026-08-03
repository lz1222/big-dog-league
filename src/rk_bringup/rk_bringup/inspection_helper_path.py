"""正式 inspection SDK helper 的安装树路径解析。

本模块不启动进程，也不导入 ROS。它把 production helper 的路径校验留在
纯函数中，避免 launch 因 basename、当前工作目录或 YAML 覆盖而把不确定的
可执行文件交给警示牌动作执行器。
"""

import os


SDK_HELPER_PACKAGE = 'rk_go2_sdk_bridge'
SDK_HELPER_BASENAME = 'go2_sdk_motion_action'


class InspectionHelperPathError(RuntimeError):
    """生产 helper 不满足绝对安装树可执行文件契约时的 fail-closed 错误。"""


def validate_absolute_executable(path):
    """返回规范化绝对可执行文件路径，任何不完整路径均拒绝。

    保留 ament 安装树中的原始绝对路径。``--symlink-install`` 时 ``realpath``
    会错误地把 ``install/...`` 改写为 ``build/...``；因此这里只规范化 ``..``
    并验证文件属性，不能依赖 PATH 或当前目录搜索。
    """
    configured = str(path or '').strip()
    if not configured or not os.path.isabs(configured):
        raise InspectionHelperPathError('sdk_action_executable_must_be_absolute')
    normalized = os.path.normpath(configured)
    if normalized != configured:
        raise InspectionHelperPathError('sdk_action_executable_must_be_normalized')
    if not os.path.isfile(normalized):
        raise InspectionHelperPathError('sdk_action_executable_not_file')
    if not os.access(normalized, os.X_OK):
        raise InspectionHelperPathError('sdk_action_executable_not_executable')
    return normalized


def resolve_production_sdk_action_helper(package_prefix):
    """从 ament 包安装前缀构造并验证 production SDK helper 的绝对路径。"""
    prefix = str(package_prefix or '').strip()
    if not prefix or not os.path.isabs(prefix):
        raise InspectionHelperPathError('sdk_helper_package_prefix_must_be_absolute')
    candidate = os.path.join(
        os.path.normpath(prefix), 'lib', SDK_HELPER_PACKAGE,
        SDK_HELPER_BASENAME,
    )
    return validate_absolute_executable(candidate)


def select_sdk_action_helper(
    hardware_mode, software_smoke_mode, fake_helper, package_prefix
):
    """选择 helper：software smoke 从不解析或触碰真实 production 路径。"""
    hardware_enabled = str(hardware_mode).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )
    smoke_enabled = str(software_smoke_mode).strip().lower() in (
        '1', 'true', 'yes', 'on'
    )
    if hardware_enabled and not smoke_enabled:
        return resolve_production_sdk_action_helper(package_prefix)
    return validate_absolute_executable(fake_helper)
