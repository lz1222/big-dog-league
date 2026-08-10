"""国赛唯一步态 owner 的纯状态机。

本模块不导入 ROS 或 SDK，只表达省赛黄金合同的调用次序和 fail-closed
边界。实际 backend 必须逐项回报 ret；任何异常都保持普通 Move 锁定。
"""


INIT = 'INIT'
ENTERING_CLASSIC = 'ENTERING_CLASSIC'
CLASSIC = 'CLASSIC'
SPECIAL_GAIT = 'SPECIAL_GAIT'
SPECIAL_ACTION = 'SPECIAL_ACTION'
RESTORING_CLASSIC = 'RESTORING_CLASSIC'
FAULT = 'FAULT'


class CompetitionGaitManagerCore:
    """串行管理经典/特殊步态，禁止恢复路径隐式改变 MotionSwitcher 服务。"""

    def __init__(self):
        self.state = INIT

    @property
    def ordinary_move_allowed(self):
        """只有经典已验证时才解除普通 Move 门，未知状态一律零速。"""
        return self.state == CLASSIC

    def begin_startup(self, check_ret, check_form, check_name):
        """读取模式后请求 normal；mcf 是有效观测，绝不产生 Release 动作。"""
        if self.state != INIT or check_ret != 0 or check_form != '0':
            return self._fault('motion_switcher_check_failed')
        self.state = ENTERING_CLASSIC
        return self._event('SELECT_NORMAL_ONCE', observed_name=check_name)

    def select_normal_result(self, ret, already_equivalent=False):
        """SelectMode 只允许一次；非零仅在已验证等价环境时可继续。"""
        if self.state != ENTERING_CLASSIC:
            return self._fault('select_normal_out_of_sequence')
        if ret != 0 and not already_equivalent:
            return self._fault('select_normal_failed_{}'.format(ret))
        return self._event('STAND_UP_IF_NEEDED')

    def stand_up_result(self, needed, ret=0):
        """站立由实体状态判断；需要而失败时不得尝试其它恢复手段。"""
        if self.state != ENTERING_CLASSIC:
            return self._fault('stand_up_out_of_sequence')
        if needed and ret != 0:
            return self._fault('stand_up_failed_{}'.format(ret))
        return self._event('SPEED_LEVEL_1_ONCE')

    def speed_level_result(self, ret):
        if self.state != ENTERING_CLASSIC:
            return self._fault('speed_level_out_of_sequence')
        if ret != 0:
            return self._fault('speed_level_failed_{}'.format(ret))
        return self._event('CLASSIC_WALK_ONCE')

    def classic_walk_result(self, ret):
        if self.state not in (ENTERING_CLASSIC, RESTORING_CLASSIC):
            return self._fault('classic_walk_out_of_sequence')
        if ret != 0:
            return self._fault('classic_walk_failed_{}'.format(ret))
        return self._event('VERIFY_CLASSIC')

    def classic_verified(self, verified):
        """静态/动态谓词由 backend 提供；未验证时普通 Move 继续锁定。"""
        if self.state not in (ENTERING_CLASSIC, RESTORING_CLASSIC):
            return self._fault('classic_verify_out_of_sequence')
        if not verified:
            return self._fault('classic_signature_not_verified')
        self.state = CLASSIC
        return self._event('CLASSIC_READY')

    def begin_special_action(self):
        if self.state != CLASSIC:
            return self._fault('special_action_without_classic')
        self.state = SPECIAL_ACTION
        return self._event('ZERO_THEN_SPECIAL_ACTION')

    def begin_special_gait(self):
        if self.state != CLASSIC:
            return self._fault('special_gait_without_classic')
        self.state = SPECIAL_GAIT
        return self._event('ZERO_THEN_SPECIAL_GAIT')

    def special_complete(self):
        if self.state not in (SPECIAL_ACTION, SPECIAL_GAIT):
            return self._fault('special_complete_out_of_sequence')
        self.state = RESTORING_CLASSIC
        return self._event('ZERO_STOP_MOVE_THEN_CLASSIC_WALK_ONCE')

    def _fault(self, reason):
        self.state = FAULT
        return self._event('FAULT_ZERO_MOVE', reason=reason)

    def _event(self, action, **fields):
        event = {'state': self.state, 'action': action,
                 'ordinary_move_allowed': self.ordinary_move_allowed}
        event.update(fields)
        return event
