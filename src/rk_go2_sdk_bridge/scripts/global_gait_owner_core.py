"""全局步态 owner 的纯状态机，不依赖 ROS 或 Unitree SDK。"""


UNKNOWN = 'UNKNOWN'
CLASSIC_REQUESTED = 'CLASSIC_REQUESTED'
CLASSIC_COMMAND_ACK = 'CLASSIC_COMMAND_ACK'
CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE = (
    'CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE'
)
CLASSIC_VALIDATED_SEQUENCE_SOURCE = (
    'validated_sequence_current_cpp_pre_stop_speed_classic_settle_v1'
)
FREE_REQUESTED = 'FREE_REQUESTED'
FREE_READY = 'FREE_READY'
FAILED = 'FAILED'


class SdkStatusSequenceGuard:
    """只接收当前 server 的严格递增状态，保留 replay 但拒绝重复倒灌。"""

    def __init__(self, expected_server_instance_id):
        self.expected_server_instance_id = str(expected_server_instance_id)
        self.last_sequence = 0

    def accept(self, server_instance_id, sequence):
        """同一实例仅允许单调递增序号，旧 replay 不得回退 owner 状态。"""
        if str(server_instance_id) != self.expected_server_instance_id:
            return False
        try:
            sequence = int(sequence)
        except (TypeError, ValueError):
            return False
        if sequence <= self.last_sequence:
            return False
        self.last_sequence = sequence
        return True


class GlobalGaitOwnerCore:
    """跟踪唯一在途请求；Classic 只接受已实机验收的固定调用序列。"""

    def __init__(self):
        self.state = UNKNOWN
        self.request_id = ''
        self.target = ''
        self.failure_reason = ''
        self.verification_source = 'none'

    def observe_startup_classic(self, ret, verification_source):
        """command ACK 不得放行；启动只接受已验收调用序列的明确来源。"""
        if int(ret) != 0:
            return self.fail('startup_classic_ret={}'.format(int(ret)))
        if verification_source != CLASSIC_VALIDATED_SEQUENCE_SOURCE:
            return self.fail('startup_classic_sequence_not_validated')
        self.state = CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE
        self.target = 'CLASSIC'
        self.request_id = ''
        self.failure_reason = ''
        self.verification_source = verification_source
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

    def observe_classic_command_ack(self, ret, reason):
        """记录中间 RPC ACK；它不能改写为实体就绪或解除 movement lock。"""
        fields = {}
        for item in str(reason).split(';'):
            if '=' in item:
                key, value = item.split('=', 1)
                fields[key] = value
        if (self.target != 'CLASSIC'
                or fields.get('request_id') != self.request_id):
            return False
        if int(ret) != 0 or fields.get('verification_source') != 'command_ack':
            return self.fail('classic_command_ack_invalid')
        self.state = CLASSIC_COMMAND_ACK
        self.verification_source = 'command_ack'
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
            if fields.get('verification_source') != CLASSIC_VALIDATED_SEQUENCE_SOURCE:
                return self.fail('classic_ack_sequence_not_validated')
            self.state = CLASSIC_ESTABLISHED_BY_VALIDATED_SEQUENCE
        elif self.target == 'FREE':
            self.state = FREE_READY
        else:
            self.state = UNKNOWN
        self.verification_source = fields.get('verification_source', 'none')
        return True

    def fail(self, reason):
        self.state = FAILED
        self.failure_reason = str(reason)
        self.verification_source = 'none'
        return False
