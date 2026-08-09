"""单次动态验收 adapter 的纯时序与 recorder 统计。

不依赖 ROS；用于证明 arm 前不会向 mux 输入队列塞入零命令，并使白横杆
时间对齐计算可以在离线回归中重复验证。
"""


def adapter_schedule(arm_ns, motion_ns, period_ns=20_000_000):
    """产生 adapter 应发布的时间表：arm 前沉默，运动结束只发布一个零。"""
    events = []
    timestamp = int(arm_ns)
    end = timestamp + int(motion_ns)
    while timestamp < end:
        events.append((timestamp, 'MOVE'))
        timestamp += int(period_ns)
    events.append((end, 'ZERO'))
    return events


def sdk_status_event_key(payload):
    """返回 SDK 事件稳定键；forwarder 重放同一 sequence 时用于去重汇总。"""
    if not isinstance(payload, dict):
        return None
    instance_id = str(payload.get('server_instance_id', '')).strip()
    sequence = payload.get('sequence')
    if not instance_id or isinstance(sequence, bool):
        return None
    try:
        sequence = int(sequence)
    except (TypeError, ValueError):
        return None
    return instance_id, sequence


def sdk_event_monotonic_ns(payload, receive_monotonic_ns):
    """优先采用 server monotonic；缺失时才保守回退 recorder 接收时钟。"""
    value = payload.get('server_monotonic_ns') if isinstance(payload, dict) else None
    if not isinstance(value, bool):
        try:
            value = int(value)
        except (TypeError, ValueError):
            value = 0
    if value > 0:
        return value
    return int(receive_monotonic_ns)


def legacy_line_stale_event(rows, timeout_ns=350_000_000):
    """重放旧 receiver-age 判定，返回首个超时相邻样本及旧 stop reason。"""
    samples = sorted(
        (
            int(row['monotonic_ns']) for row in rows
            if row.get('channel') == 'line_track'
        )
    )
    for previous, following in zip(samples, samples[1:]):
        if following - previous > int(timeout_ns):
            return {
                'previous_ns': previous,
                'next_ns': following,
                'gap_ns': following - previous,
                'stop_reason': 'LINE_TRACK_FAILURE',
            }
    return None


def nearest_center_y(frames, timestamp_ns):
    """返回距指定 SDK 时间最近的可靠白横杆中心；无证据时保持 None。"""
    candidates = [row for row in frames if row['visible'] and row['confidence'] > 0.0]
    if not candidates or timestamp_ns is None:
        return None
    return min(candidates, key=lambda row: abs(row['monotonic_ns'] - timestamp_ns))['center_y']


def white_bar_metrics(frames, t0_ns, tstop_ns, stable_frames=3):
    """从逐帧记录计算可审计的白横杆可见性、稳定首帧和最长丢失。"""
    reliable = [row for row in frames if row['visible'] and row['confidence'] > 0.0]
    first_stable = None
    for index in range(len(frames) - stable_frames + 1):
        window = frames[index:index + stable_frames]
        if all(row['visible'] and row['confidence'] > 0.0 for row in window):
            first_stable = window[0]['monotonic_ns']
            break
    longest_loss_ns = 0
    loss_start = None
    for row in frames:
        if not row['visible'] and loss_start is None:
            loss_start = row['monotonic_ns']
        elif row['visible'] and loss_start is not None:
            longest_loss_ns = max(longest_loss_ns, row['monotonic_ns'] - loss_start)
            loss_start = None
    if loss_start is not None and frames:
        longest_loss_ns = max(longest_loss_ns, frames[-1]['monotonic_ns'] - loss_start)
    centers = [row['center_y'] for row in reliable]
    return {
        'first_visible_ns': reliable[0]['monotonic_ns'] if reliable else None,
        'first_stable_visible_ns': first_stable,
        'center_y_t0': nearest_center_y(frames, t0_ns),
        'center_y_min': min(centers) if centers else None,
        'center_y_max': max(centers) if centers else None,
        'center_y_tstop': nearest_center_y(frames, tstop_ns),
        'last_reliable_visible_ns': reliable[-1]['monotonic_ns'] if reliable else None,
        'longest_loss_sec': longest_loss_ns / 1e9,
    }
