#!/usr/bin/env python3

"""B2 终端 D 中文操作提示核心，不依赖 ROS 或硬件接口。"""

from dataclasses import dataclass
import math


STATE_LABELS = {
    'WAIT_SENSOR': '等待传感器',
    'CORRIDOR_FOLLOW': '走廊跟随',
    'CORNER_APPROACH': '接近拐角',
    'TURN_LEFT': '向左转弯',
    'TURN_RIGHT': '向右转弯',
    'TURN_FINE_ALIGN': '转后精调',
    'CORRIDOR_REACQUIRE': '重新进入走廊',
    'REVERSE_RECOVERY': '倒退恢复候选',
    'FINISHED': '迷宫路线完成',
    'FAULT_STOP': '故障锁止',
}

DIRECTION_LABELS = {
    'LEFT': '左转',
    'RIGHT': '右转',
}

CENTER_REFERENCE_LABELS = {
    'both_walls': '左右墙',
    'left_wall': '左墙',
    'right_wall': '右墙',
    'none': '无',
}

REASON_LABELS = {
    'startup_wait_sensor': '节点刚启动，正在等待传感器',
    'b1_input_missing': '尚未收到 B1 状态',
    'b1_input_stale': 'B1 状态 Topic 已中断',
    'b1_sensor_stale': 'B1 判定传感器数据过期',
    'b1_sensor_state_invalid': 'B1 状态值无效',
    'cloud_age_invalid': '点云时间戳无效',
    'cloud_stale': '点云数据过期',
    'odom_age_invalid': '里程计时间戳无效',
    'odom_stale': '里程计数据过期',
    'yaw_invalid': 'Yaw 数据无效',
    'turn_invalid': '累计转角数据无效',
    'distance_invalid': '五扇区距离数据无效',
    'side_distance_confirmation_pending': '左右侧墙距离尚未稳定',
    'sensors_confirmed': '传感器连续确认完成',
    'corridor_centering': '正在根据左右墙距离保持居中',
    'route_complete_search_exit': '五次转向完成，正在确认出口开放',
    'corner_distance_confirmed': '已确认接近拐角',
    'approaching_turn_start': '尚未到达允许转向的位置',
    'waiting_for_turn_opening': '已到转向距离，但转向扫掠空间不足',
    'turn_sweep_unsafe': '转向扫掠包络不足',
    'fine_align_sweep_unsafe': '精调扫掠包络不足',
    'left_turn_started': '左转条件已满足',
    'right_turn_started': '右转条件已满足',
    'yaw_closed_loop_turn': '正在使用累计 Yaw 闭环转向',
    'enter_turn_fine_align': '已接近目标角，进入小幅精调',
    'fine_align_error_increased': '角度误差增大，返回转弯状态',
    'turn_angle_confirmed': '目标转角已连续确认',
    'corridor_reacquired': '新走廊已重新确认',
    'front_emergency_reverse_candidate': '前方距离过近，进入恢复候选',
    'turn_envelope_unsafe_reverse_candidate': '转向空间不足，进入恢复候选',
    'reverse_recovery_clearance_restored': '前方和转向空间已恢复',
    'reverse_recovery_diagnostic_only_no_rear_sector': (
        'B1 没有后向扇区，禁止按诊断值直接倒退'
    ),
    'radar_exit_confirmed': '雷达已连续确认出口开放',
    'corner_approach_timeout': '接近拐角超时',
    'corner_without_expected_turn': '路线中没有可用的下一转向',
    'corner_side_distance_missing': '拐角所需侧向距离缺失',
    'corner_side_clearance_unsafe': '拐角侧向安全余量不足',
    'corridor_side_distance_missing': '走廊侧墙距离缺失',
    'corridor_side_clearance_unsafe': '走廊侧向安全余量不足',
    'turn_timeout': '转向超时',
    'turn_clearance_missing': '转向所需空间数据缺失',
    'turn_clearance_unsafe': '转向瞬时安全余量不足',
    'turn_target_missing': '转向目标或累计转角缺失',
    'turn_fine_align_timeout': '转向精调超时',
    'fine_align_clearance_missing': '精调所需空间数据缺失',
    'fine_align_clearance_unsafe': '精调时安全余量不足',
    'fine_align_target_missing': '精调目标或累计转角缺失',
    'corridor_reacquire_timeout': '重新进入走廊超时',
    'reacquire_side_distance_missing': '新走廊侧墙距离缺失',
    'reacquire_side_clearance_unsafe': '新走廊侧向安全余量不足',
    'reacquire_front_clearance_unsafe': '新走廊前方安全余量不足',
    'reverse_recovery_timeout': '恢复候选超时',
    'reverse_side_distance_missing': '恢复阶段侧向距离缺失',
    'reverse_side_clearance_unsafe': '恢复阶段侧向安全余量不足',
}


@dataclass(frozen=True)
class OperatorView:
    """描述操控员此刻应执行的唯一动作及其安全等级。"""

    severity: str
    action_code: str
    action_title: str
    instruction: str
    state_label: str
    reason_label: str


def build_operator_view(
    payload,
    stream_age_sec,
    stale_timeout_sec,
    manual_step_distance_cm=8.0,
):
    """将一帧 B2 JSON 转换为保守的中文人工操作提示。"""
    if not isinstance(payload, dict):
        return _stop_view(
            '等待 B2 状态',
            '保持静止；确认 B1、B2 和终端 D 均已启动。',
            '尚未收到有效的 B2 JSON',
            severity='warning',
        )

    step_distance = _finite_number(manual_step_distance_cm)
    if step_distance is None or step_distance <= 0.0:
        return _stop_view(
            '人工点动标定无效',
            '保持静止；检查 manual_step_distance_cm 参数。',
            '无法生成可执行的人工前进提示',
            severity='warning',
        )

    age = _finite_number(stream_age_sec)
    timeout = _finite_number(stale_timeout_sec)
    if age is None or timeout is None or timeout <= 0.0:
        return _stop_view(
            '状态时间无效',
            '立即松开遥控器并停止；检查终端 D 参数。',
            '无法判断 B2 状态是否新鲜',
        )
    if age > timeout:
        last_state = str(payload.get('state', '')).upper()
        last_state_label = STATE_LABELS.get(
            last_state,
            last_state or '未知',
        )
        last_reason = translate_reason(payload.get('reason', ''))
        return _stop_view(
            f'B2 状态中断（最后：{last_state_label}）',
            '立即松开遥控器并停止；恢复状态流后重启 B2。',
            (
                f'超过 {timeout:.2f}s 未收到 B2 JSON；'
                f'最后原因：{last_reason}'
            ),
        )

    state = str(payload.get('state', '')).upper()
    reason = str(payload.get('reason', ''))
    state_label = STATE_LABELS.get(state, f'未知状态 {state or "空值"}')
    reason_label = translate_reason(reason)

    if 'missing_confirmation_' in reason:
        return _stop_view(
            state_label,
            '立即松开遥控器并保持静止；等待侧向距离恢复，禁止继续前进或转向。',
            reason_label,
            severity='warning',
        )
    if 'clearance_unsafe_confirmation_' in reason:
        return _stop_view(
            state_label,
            '立即松开遥控器并保持静止；等待下一帧确认侧向安全余量。',
            reason_label,
            severity='danger',
        )

    turning_states = ('TURN_LEFT', 'TURN_RIGHT', 'TURN_FINE_ALIGN')
    if (
        state in turning_states
        and payload.get('moving_turn_sweep_safe') is not True
    ):
        # D端独立执行最后一道保护，绝不在包络不足时提示操控员转向。
        return _stop_view(
            state_label,
            '保持静止；当前矩形机身转向扫掠包络不足，禁止继续转向。',
            reason_label,
            severity='danger',
        )

    if state == 'WAIT_SENSOR':
        return OperatorView(
            'warning',
            'HOLD',
            '保持静止',
            '不要推动遥控杆；等待点云、Odom 和左右侧距连续确认。',
            state_label,
            reason_label,
        )
    if state == 'CORRIDOR_FOLLOW':
        if bool(payload.get('route_complete')):
            instruction = (
                '执行一次最短可控前进点动'
                f'（当前标定约 {step_distance:g}cm），随即松杆观察；'
                '等待雷达确认出口。'
            )
        else:
            instruction = (
                '执行一次最短可控前进点动'
                f'（当前标定约 {step_distance:g}cm），随即松杆观察；'
                '禁止连续点动，并根据左右侧距保持居中。'
            )
        return OperatorView(
            'normal',
            'FORWARD_STEP',
            '短促前进',
            instruction,
            state_label,
            reason_label,
        )
    if state == 'CORNER_APPROACH':
        if reason.startswith('turn_start_confirmation_'):
            return _stop_view(
                state_label,
                '保持静止；等待转向开口和扫掠包络连续确认。',
                reason_label,
                severity='warning',
            )
        if reason == 'waiting_for_turn_opening':
            return _stop_view(
                state_label,
                '立即停止前进，不要提前转向；记录距离并检查开口侧。',
                reason_label,
                severity='warning',
            )
        return OperatorView(
            'caution',
            'FORWARD_FINE',
            '更小步前进',
            '仅执行一次最短可控前进点动'
            f'（当前标定约 {step_distance:g}cm），随即松杆；'
            '等待明确的 TURN_LEFT 或 TURN_RIGHT。',
            state_label,
            reason_label,
        )
    if state == 'TURN_LEFT':
        return OperatorView(
            'turn',
            'TURN_LEFT',
            '开始左转',
            '缓慢执行“少量前进 + 左转”；禁止纯原地旋转，持续观察转角误差。',
            state_label,
            reason_label,
        )
    if state == 'TURN_RIGHT':
        return OperatorView(
            'turn',
            'TURN_RIGHT',
            '开始右转',
            '缓慢执行“少量前进 + 右转”；禁止纯原地旋转，持续观察转角误差。',
            state_label,
            reason_label,
        )
    if state == 'TURN_FINE_ALIGN':
        return _fine_align_view(payload, state_label, reason_label)
    if state == 'CORRIDOR_REACQUIRE':
        return OperatorView(
            'caution',
            'REACQUIRE',
            '缓慢进入新通道',
            '仅执行一次最短可控前进点动'
            f'（当前标定约 {step_distance:g}cm）并小幅居中；'
            '出现下一段 CORRIDOR_FOLLOW 后松杆停止。',
            state_label,
            reason_label,
        )
    if state == 'REVERSE_RECOVERY':
        return _stop_view(
            state_label,
            '立即停止；B1 没有后向视野，禁止按 desired_vx 倒退，改由人工检查位置。',
            reason_label,
        )
    if state == 'FINISHED':
        return OperatorView(
            'complete',
            'FINISHED',
            '路线完成，保持停止',
            '松开遥控器并保持站立；停止录包后再结束 B2。',
            state_label,
            reason_label,
        )
    if state == 'FAULT_STOP':
        return _stop_view(
            state_label,
            '立即松开遥控器并停止；排除原因后必须重启 B2，不能在锁止状态继续。',
            reason_label,
        )

    return _stop_view(
        state_label,
        '立即松开遥控器并停止；未知状态不得继续人工行走。',
        reason_label,
    )


def translate_reason(reason):
    """翻译固定原因，并保留持续帧计数和原始未知文本。"""
    text = str(reason or '')
    if text in REASON_LABELS:
        return REASON_LABELS[text]
    if 'missing_confirmation_' in text:
        base_reason, count = text.rsplit('_confirmation_', 1)
        base_label = REASON_LABELS.get(base_reason, base_reason)
        return f'{base_label}，暂停确认 {count}'
    if 'clearance_unsafe_confirmation_' in text:
        base_reason, count = text.rsplit('_confirmation_', 1)
        base_label = REASON_LABELS.get(base_reason, base_reason)
        return f'{base_label}，危险帧确认 {count}'
    if 'sweep_unsafe_confirmation_' in text:
        base_reason, count = text.rsplit('_confirmation_', 1)
        base_label = REASON_LABELS.get(base_reason, base_reason)
        return f'{base_label}，危险帧确认 {count}'
    prefixes = (
        ('sensor_confirmation_', '传感器连续确认 '),
        ('turn_start_confirmation_', '转向开口连续确认 '),
        ('fine_align_confirmation_', '目标角连续确认 '),
        ('corridor_reacquire_', '新走廊连续确认 '),
        ('b1_payload_invalid:', 'B1 JSON 格式错误：'),
        ('unhandled_state_', 'B2 未处理状态：'),
    )
    for prefix, label in prefixes:
        if text.startswith(prefix):
            return label + text[len(prefix):]
    return text or '未提供原因'


def format_dashboard(payload, view, stream_age_sec):
    """生成固定宽度的中文状态面板，所有速度均标明为只读诊断值。"""
    data = payload if isinstance(payload, dict) else {}
    route_index = _integer_or_default(data.get('route_index'), 0)
    route_total = _integer_or_default(data.get('route_total'), 0)
    expected_turn = DIRECTION_LABELS.get(
        str(data.get('expected_turn', '')).upper(),
        '无',
    )
    distances = data.get('distances_m')
    if not isinstance(distances, dict):
        distances = {}
    center_reference = CENTER_REFERENCE_LABELS.get(
        str(data.get('center_reference', 'none')),
        str(data.get('center_reference', '未知')),
    )

    lines = [
        '=' * 72,
        'B2 迷宫人工干跑 - 终端 D（只读提示，不发送任何运动命令）',
        f'当前指令：【{view.action_title}】  安全级别：{_severity_label(view.severity)}',
        f'操作说明：{view.instruction}',
        '-' * 72,
        (
            f'状态：{view.state_label} ({data.get("state", "n/a")})  '
            f'状态持续：{_format_value(data.get("state_age_sec"), 1, "s")}'
        ),
        f'原因：{view.reason_label} ({data.get("reason", "n/a")})',
        (
            f'路线：已完成 {route_index}/{route_total} 次转向  '
            f'下一大方向：{expected_turn}'
        ),
        (
            '距离(m)：'
            f'前 {_format_value(distances.get("front"), 3)}  '
            f'左前 {_format_value(distances.get("left_front"), 3)}  '
            f'右前 {_format_value(distances.get("right_front"), 3)}  '
            f'左 {_format_value(distances.get("left"), 3)}  '
            f'右 {_format_value(distances.get("right"), 3)}'
        ),
        (
            '姿态：'
            f'Yaw {_format_degrees_from_rad(data.get("yaw_rad"))}  '
            'Odom启动累计 '
            f'{_format_degrees_from_rad(data.get("turn_rad"))}（含静止漂移）'
        ),
        (
            '转弯闭环：'
            f'本次进度 {_format_value(data.get("turn_progress_deg"), 1, "deg")}  '
            f'目标误差 {_format_value(data.get("turn_error_deg"), 1, "deg")}'
        ),
        (
            f'居中（{center_reference}参考）：'
            f'{_format_value(data.get("center_error_m"), 3, "m")} '
            '(正值表示应向左修正，负值表示应向右修正)'
        ),
        (
            '转向包络：'
            f'{_format_bool(data.get("moving_turn_sweep_safe"))}  '
            '启动条件：'
            f'{_format_bool(data.get("turn_start_sweep_safe"))}  '
            '开口滞回：'
            f'{_format_latched(data.get("turn_open_latched"))}  '
            '原地转向：'
            f'{_in_place_label(data.get("in_place_rotation_fits_corridor"))}'
        ),
        (
            '数据新鲜度：'
            f'B2 {_format_value(stream_age_sec, 3, "s")}  '
            f'点云 {_format_value(data.get("cloud_age_sec"), 3, "s")}  '
            f'Odom {_format_value(data.get("odom_age_sec"), 3, "s")}'
        ),
        (
            '诊断候选（不会发送）：'
            f'vx={_format_value(data.get("desired_vx"), 3)}  '
            f'wz={_format_value(data.get("desired_wz"), 3)}'
        ),
        '=' * 72,
    ]
    return '\n'.join(lines)


def _fine_align_view(payload, state_label, reason_label):
    error = _finite_number(payload.get('turn_error_deg'))
    tolerance = _finite_number(payload.get('turn_tolerance_deg'))
    if tolerance is None or tolerance <= 0.0:
        # 兼容没有该诊断字段的旧 B2 录包。
        tolerance = 4.0
    if error is None:
        return _stop_view(
            state_label,
            '立即停止；转角误差不可用，等待 B2 进入故障保护。',
            reason_label,
        )
    if abs(error) <= tolerance:
        return OperatorView(
            'warning',
            'HOLD_ALIGN',
            '松杆保持，等待角度确认',
            '不要继续增加转角；保持静止，等待连续帧进入新走廊确认。',
            state_label,
            reason_label,
        )
    direction = '左' if error > 0.0 else '右'
    return OperatorView(
        'caution',
        'FINE_ALIGN',
        f'向{direction}小幅精调',
        f'仅向{direction}轻微转动并保留少量前进；误差接近 0deg 后立即松杆。',
        state_label,
        reason_label,
    )


def _stop_view(state_label, instruction, reason_label, severity='danger'):
    return OperatorView(
        severity,
        'STOP',
        '立即停止' if severity == 'danger' else '停止并等待',
        instruction,
        state_label,
        reason_label,
    )


def _finite_number(value):
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer_or_default(value, default):
    number = _finite_number(value)
    if number is None:
        return int(default)
    return int(number)


def _format_value(value, precision, suffix=''):
    number = _finite_number(value)
    if number is None:
        return 'n/a'
    return f'{number:.{precision}f}{suffix}'


def _format_degrees_from_rad(value):
    number = _finite_number(value)
    if number is None:
        return 'n/a'
    return f'{math.degrees(number):.1f}deg'


def _format_bool(value):
    if value is True:
        return '安全'
    if value is False:
        return '不足'
    return '未知'


def _format_latched(value):
    """将开口滞回状态与普通安全布尔量区分显示。"""
    if value is True:
        return '已锁存'
    if value is False:
        return '未锁存'
    return '未知'


def _in_place_label(value):
    if value is True:
        return '几何允许'
    if value is False:
        return '禁止'
    return '未知'


def _severity_label(severity):
    return {
        'danger': '危险/必须停止',
        'warning': '保持/等待',
        'caution': '低速谨慎操作',
        'turn': '允许转向',
        'normal': '允许短促前进',
        'complete': '完成/保持停止',
    }.get(str(severity), '未知/保持停止')
