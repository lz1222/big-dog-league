#!/usr/bin/env python3
"""以确定的 Unitree DDS 运行时启动 SDK UDP server。

该 wrapper 仅修改即将 exec 的 SDK 子进程环境，不影响 ROS Foxy 进程。它要求
libddsc 和 libddscxx 位于同一个构建时确认的目录，避免 ROS 的 CycloneDDS
动态库与 Unitree SDK 的 C++ 包装库混用。
"""

import os
import sys


def _runtime_dir_from_install(script_path):
    """相对 wrapper 自身读取 SDK 运行库目录，安装后不依赖源码树。"""
    config_path = os.path.join(
        os.path.dirname(os.path.abspath(script_path)), 'go2_sdk_runtime_dir.txt'
    )
    with open(config_path, 'r') as config_file:
        runtime_dir = config_file.read().strip()
    if not runtime_dir or os.path.isabs(runtime_dir):
        raise ValueError('SDK runtime directory configuration is empty')
    return os.path.join(os.path.dirname(os.path.abspath(script_path)), runtime_dir)


def build_sdk_server_environment(runtime_dir, inherited_environment):
    """返回隔离后的 SDK 环境，禁止继承任意 CycloneDDS 搜索路径。

    CYCLONEDDS_URI 保留给 SDK 的网络配置；ROS_DOMAIN_ID 不参与此处的
    ChannelFactory domain，后者始终由 server 源码中的 0 决定。
    """
    runtime_dir = os.path.abspath(runtime_dir)
    required_libraries = ('libddsc.so.0', 'libddscxx.so.0')
    missing = [
        library for library in required_libraries
        if not os.path.isfile(os.path.join(runtime_dir, library))
    ]
    if missing:
        raise ValueError(
            'SDK runtime directory must contain one compatible libddsc/libddscxx '
            'pair: {}'.format(', '.join(missing))
        )

    environment = dict(inherited_environment)
    # 只允许这一对 SDK 库参与解析；空或混合的父路径都不能漏入子进程。
    environment['LD_LIBRARY_PATH'] = runtime_dir
    environment.pop('LD_PRELOAD', None)
    # 本地 SDK 示例明确支持 CYCLONEDDS_URI；历史成功环境没有完整快照，
    # 因而默认保留调用方配置，并允许用专用变量显式覆盖或置空取消继承。
    uri_override = environment.pop('RK_UNITREE_SDK_CYCLONEDDS_URI', None)
    if uri_override is not None:
        if uri_override:
            environment['CYCLONEDDS_URI'] = uri_override
        else:
            environment.pop('CYCLONEDDS_URI', None)
    return environment


def main(argv):
    """校验安装树配置后 exec 原 server；校验失败时以非零状态 fail-closed。"""
    if len(argv) < 2:
        raise ValueError('usage: go2_sdk_server_runtime.py SDK_SERVER [ARGS...]')

    runtime_dir = _runtime_dir_from_install(argv[0])
    environment = build_sdk_server_environment(runtime_dir, os.environ)
    server_path = os.path.abspath(argv[1])
    if not os.path.isfile(server_path) or not os.access(server_path, os.X_OK):
        raise ValueError('SDK server is not an executable file: {}'.format(server_path))

    # execve 替换 wrapper：父进程监控到的 PID/信号/进程组仍属于真实 SDK helper。
    os.execve(server_path, argv[1:], environment)


if __name__ == '__main__':
    try:
        main(sys.argv)
    except (OSError, ValueError) as error:
        print('[SDK-RUNTIME] fatal: {}'.format(error), file=sys.stderr)
        sys.exit(125)
