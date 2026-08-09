#!/usr/bin/env python3
"""兼容入口：仅启动独立 validation motion adapter，不再承担 recorder。"""

from rk_bringup.validation_motion_adapter_node import main


if __name__ == '__main__':
    main()
