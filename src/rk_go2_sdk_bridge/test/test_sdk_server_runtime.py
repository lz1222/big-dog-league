"""Unitree SDK server 运行时隔离的纯软件回归测试。"""

import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WRAPPER_PATH = PACKAGE_ROOT / 'scripts' / 'go2_sdk_server_runtime.py'


def _load_wrapper_module():
    spec = importlib.util.spec_from_file_location(
        'go2_sdk_server_runtime', str(WRAPPER_PATH)
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sdk_environment_replaces_mixed_cyclonedds_paths(tmp_path):
    """SDK 子进程不得继承 ROS 的混合 libddsc/libddscxx 搜索路径。"""
    runtime_dir = tmp_path / 'unitree-sdk-runtime'
    runtime_dir.mkdir()
    for library in ('libddsc.so.0', 'libddscxx.so.0'):
        (runtime_dir / library).touch()

    wrapper = _load_wrapper_module()
    environment = wrapper.build_sdk_server_environment(
        str(runtime_dir),
        {
            'LD_LIBRARY_PATH': '/usr/local/cyclonedds/lib:/usr/local/lib',
            'LD_PRELOAD': '/usr/local/lib/libddsc.so',
            'CYCLONEDDS_URI': '/tmp/cyclonedds.xml',
            'ROS_DOMAIN_ID': '10',
        },
    )

    assert environment['LD_LIBRARY_PATH'] == str(runtime_dir)
    assert 'LD_PRELOAD' not in environment
    assert environment['CYCLONEDDS_URI'] == '/tmp/cyclonedds.xml'
    assert environment['ROS_DOMAIN_ID'] == '10'


def test_sdk_environment_allows_explicit_cyclonedds_uri_policy(tmp_path):
    """URI 默认继承历史 shell，专用变量可显式覆盖或取消继承。"""
    runtime_dir = tmp_path / 'unitree-sdk-runtime'
    runtime_dir.mkdir()
    for library in ('libddsc.so.0', 'libddscxx.so.0'):
        (runtime_dir / library).touch()

    wrapper = _load_wrapper_module()
    overridden = wrapper.build_sdk_server_environment(
        str(runtime_dir), {
            'CYCLONEDDS_URI': '/ros.xml',
            'RK_UNITREE_SDK_CYCLONEDDS_URI': '/unitree.xml',
        }
    )
    cleared = wrapper.build_sdk_server_environment(
        str(runtime_dir), {
            'CYCLONEDDS_URI': '/ros.xml',
            'RK_UNITREE_SDK_CYCLONEDDS_URI': '',
        }
    )
    assert overridden['CYCLONEDDS_URI'] == '/unitree.xml'
    assert 'CYCLONEDDS_URI' not in cleared


def test_sdk_environment_requires_complete_same_directory_pair(tmp_path):
    """缺少任一库时必须拒绝启动，不能退回到系统 CycloneDDS。"""
    runtime_dir = tmp_path / 'incomplete-runtime'
    runtime_dir.mkdir()
    (runtime_dir / 'libddsc.so.0').touch()

    wrapper = _load_wrapper_module()
    try:
        wrapper.build_sdk_server_environment(str(runtime_dir), {})
    except ValueError as error:
        assert 'libddscxx.so' in str(error)
    else:
        raise AssertionError('incomplete SDK DDS pair must fail closed')


def test_launch_uses_installed_wrapper_and_absolute_server_path():
    """生产 launch 必须经安装树 wrapper 启动，不允许直接继承 ROS 动态库环境。"""
    launch_source = (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'launch' /
        'competition_non_arm.launch.py'
    ).read_text(encoding='utf-8')

    assert 'go2_sdk_server_runtime.py' in launch_source
    assert 'cmd=[sdk_server_runtime, sdk_server]' in launch_source
    assert "'sdk_runtime_wrapper': ParameterValue(" in launch_source
    assert "'channel_factory_domain=0'" not in launch_source
    assert 'go2_sdk_udp_server' in launch_source


def test_wrapper_does_not_map_ros_domain_to_unitree_channel_domain():
    """wrapper 只隔离动态库，不能把 ROS_DOMAIN_ID 变成 Unitree Domain。"""
    source = WRAPPER_PATH.read_text(encoding='utf-8')
    server_source = (
        PACKAGE_ROOT / 'src' / 'go2_sdk_udp_server.cpp'
    ).read_text(encoding='utf-8')
    assert 'ROS_DOMAIN_ID' in source
    assert 'ROS_DOMAIN_ID' not in source.split('environment =', 1)[1]
    assert 'environment[\'LD_LIBRARY_PATH\'] = runtime_dir' in source
    assert 'ChannelFactory::Instance()->Init(\n      0, config.network_interface)' in server_source


def test_all_sdk_targets_use_origin_relative_rpath_and_runtime_wrapper():
    """server、motion 和前向相机 helper 共享安装树运行时策略。"""
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text(encoding='utf-8')
    executor_source = (
        PACKAGE_ROOT.parent / 'rk_mission' / 'rk_mission' /
        'inspection_action_executor_node.py'
    ).read_text(encoding='utf-8')
    bridge_source = (
        PACKAGE_ROOT / 'scripts' / 'go2_front_camera_bridge.py'
    ).read_text(encoding='utf-8')
    launch_source = (
        PACKAGE_ROOT.parent / 'rk_bringup' / 'launch' /
        'competition_non_arm.launch.py'
    ).read_text(encoding='utf-8')

    assert 'INSTALL_RPATH "\\$ORIGIN/${RK_UNITREE_DDS_RUNTIME_INSTALL_DIR}"' in cmake_source
    assert 'libddscxx.so.0' in cmake_source
    assert 'expected_executable=executable' in executor_source
    assert 'return [wrapper] + command' in executor_source
    assert 'command.insert(0, self._sdk_runtime_wrapper)' in bridge_source
    assert "'front_jump.sdk_runtime_wrapper': ParameterValue(" in launch_source
