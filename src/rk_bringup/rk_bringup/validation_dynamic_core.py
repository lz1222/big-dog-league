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
