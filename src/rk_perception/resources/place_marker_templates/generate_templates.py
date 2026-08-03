#!/usr/bin/env python3
"""生成开发期数字基线模板；已校准的实体模板不得用本脚本覆盖。"""

from pathlib import Path

import cv2
import numpy as np


def make_template(label):
    """生成与红环内黑色阿拉伯数字对应的白色前景二值模板。"""
    image = np.zeros((128, 128), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # 真实候选先按整个红环直径归一化，编号在 128 像素模板中约为 1.9 倍字号。
    scale = 1.9
    thickness = 5
    text_size, _ = cv2.getTextSize(label, font, scale, thickness)
    origin = ((128 - text_size[0]) // 2, (128 + text_size[1]) // 2)
    cv2.putText(image, label, origin, font, scale, 255, thickness,
                cv2.LINE_AA)
    _, image = cv2.threshold(image, 60, 255, cv2.THRESH_BINARY)
    return image


def main():
    """显式生成开发期基线，现场已校准资源仅能由采集校准流程替换。"""
    output_dir = Path(__file__).resolve().parent
    for label in ('1', '2'):
        cv2.imwrite(str(output_dir / f'place_{label}.png'),
                    make_template(label))


if __name__ == '__main__':
    main()
