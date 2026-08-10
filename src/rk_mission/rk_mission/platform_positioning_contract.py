"""任务平台与正式路线层之间的纯软件接口合同。

平台节点尚未并入生产 ``line_course_mission``。本模块先把路由消息和
FINISH 互锁的安全边界写成可独立测试的合同，避免把字段缺失或内部布尔值
误认为已经完成生产接入。
"""

import json
from dataclasses import dataclass


PLATFORM_ROUTE_PHASES = frozenset((
    'NONE',
    'TRANSFER_PLATFORM_APPROACH',
    'PICKUP_PLATFORM_APPROACH',
    'PLACE_PLATFORM_APPROACH',
))


@dataclass(frozen=True)
class PlatformRouteContract:
    """严格解析后的平台路线字段；不合法输入一律安全回落到 ``NONE``。"""

    valid: bool
    platform_route_phase: str
    place_platform_id: str
    reason: str


def parse_platform_route_state(raw_message):
    """解析路线 JSON，禁止用字段缺失隐式表示平台阶段。

    ``route_phase`` 仍由现有正式路线消费；本接口只消费新增且必填的
    ``platform_route_phase``，从而不会猜测或复用上一帧平台状态。
    """
    try:
        payload = json.loads(raw_message)
    except (TypeError, ValueError, json.JSONDecodeError):
        return PlatformRouteContract(
            False, 'NONE', '', 'route_message_malformed')
    if not isinstance(payload, dict):
        return PlatformRouteContract(
            False, 'NONE', '', 'route_message_not_object')
    phase = payload.get('platform_route_phase')
    if not isinstance(phase, str):
        return PlatformRouteContract(
            False, 'NONE', '', 'platform_route_phase_missing')
    if phase not in PLATFORM_ROUTE_PHASES:
        return PlatformRouteContract(
            False, 'NONE', '', 'platform_route_phase_invalid')
    place_id = payload.get('place_platform_id', '')
    if place_id is None:
        place_id = ''
    if not isinstance(place_id, str):
        return PlatformRouteContract(
            False, 'NONE', '', 'place_platform_id_invalid')
    return PlatformRouteContract(True, phase, place_id, 'route_contract_valid')


class PlatformFinishInterlockContract:
    """生产接入前的 FINISH 白线授权合同。

    该合同目前不直接替换 ``NonArmRoutePhaseCore`` 或
    ``WhiteBarStageController``，故状态明确标记为
    ``PRODUCTION_FINISH_INTERLOCK_PENDING``。未来路线层必须同时满足
    PLACE_DONE 与独立 REARM_FINISH 才能将 FINISH 指令交给白线阶段机。
    """

    production_status = 'PRODUCTION_FINISH_INTERLOCK_PENDING'

    def __init__(self):
        self.place_done = False
        self.finish_rearmed = False

    def observe_platform_event(self, event):
        """只接受明确的放置完成和显式 rearm，其他事件不放宽授权。"""
        if event == 'PLACE_DONE':
            self.place_done = True
            self.finish_rearmed = False
        elif event == 'REARM_FINISH' and self.place_done:
            self.finish_rearmed = True

    def finish_allowed(self):
        """返回未来 route/white-stage 接口是否可 arm FINISH。"""
        return self.place_done and self.finish_rearmed

    def can_forward_stage(self, stage):
        """FINISH 在合同满足前必须被阻断，START 不受此平台合同影响。"""
        return stage != 'FINISH' or self.finish_allowed()
