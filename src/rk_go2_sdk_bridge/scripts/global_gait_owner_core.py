"""全局步态 owner 的纯状态机，不依赖 ROS 或 Unitree SDK。"""


UNKNOWN = 'UNKNOWN'
CLASSIC_REQUESTED = 'CLASSIC_REQUESTED'
CLASSIC_READY = 'CLASSIC_READY'
FREE_REQUESTED = 'FREE_REQUESTED'
FREE_READY = 'FREE_READY'
FAILED = 'FAILED'


class GlobalGaitOwnerCore:
    """跟踪唯一在途请求；READY 只代表匹配请求的 command ACK。"""

    def __init__(self):
        self.state = UNKNOWN
        self.request_id = ''
        self.target = ''
        self.failure_reason = ''
        self.verification_source = 'none'

    def observe_startup_classic(self, ret):
        """接收 server 启动 CLASSIC ACK；非零回执必须锁存失败。"""
        if int(ret) != 0:
            return self.fail('startup_classic_ret={}'.format(int(ret)))
        self.state = CLASSIC_READY
        self.target = 'CLASSIC'
        self.request_id = ''
        self.failure_reason = ''
        self.verification_source = 'command_ack'
        return True

    def begin(self, request_id, target):
        """创建一个串行转换；调用方 mutex 保证不会覆盖另一在途请求。"""
        target = str(target).upper()
        if not request_id or target not in ('CLASSIC', 'FREE', 'HOLD'):
            return False
        self.request_id = str(request_id)
        self.target = target
        self.failure_reason = ''
        self.verification_source = 'none'
        if target == 'CLASSIC':
            self.state = CLASSIC_REQUESTED
        elif target == 'FREE':
            self.state = FREE_REQUESTED
        else:
            self.state = UNKNOWN
        return True

    def observe_ack(self, event, ret, reason):
        """只接受当前 request_id/target 的 ACK，旧包不能完成新转换。"""
        fields = {}
        for item in str(reason).split(';'):
            if '=' in item:
                key, value = item.split('=', 1)
                fields[key] = value
        if (
            fields.get('request_id') != self.request_id
            or fields.get('target') != self.target
        ):
            return False
        if event == 'GAIT_FAILED' or int(ret) != 0:
            self.fail('{} ret={}'.format(event, int(ret)))
            return True
        if event != 'GAIT_READY':
            return False
        if self.target == 'CLASSIC':
            self.state = CLASSIC_READY
        elif self.target == 'FREE':
            self.state = FREE_READY
        else:
            self.state = UNKNOWN
        self.verification_source = 'command_ack'
        return True

    def fail(self, reason):
        self.state = FAILED
        self.failure_reason = str(reason)
        self.verification_source = 'none'
        return False
