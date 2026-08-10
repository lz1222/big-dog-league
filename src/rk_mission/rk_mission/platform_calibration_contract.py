"""只读标定采集结果的有效性合同。

采集工具不能因为未检测到目标仍产出看似可用于生产的 golden JSON；本模块
把零样本和不完整样本的故障关闭规则抽成不依赖 ROS 的小函数。
"""


def pickup_capture_result(samples):
    """仅正样本计数大于零时，前置挡板采集才可供人工复核。"""
    count = sum(len(values) for values in samples.values())
    if count == 0:
        return (
            False,
            'no_positive_samples: DETECTOR_THRESHOLD_CALIBRATION_REQUIRED',
        )
    return True, 'capture_ready'


def place_capture_result(samples):
    """放置 profile 必须同时有白横线与 LineTrack 样本，缺一不可。"""
    white_count = len(samples.get('center_y_ratio', ()))
    line_count = len(samples.get('line_lateral_error', ()))
    if white_count == 0 or line_count == 0:
        return False, 'insufficient_white_bar_or_line_samples'
    return True, 'capture_ready'
