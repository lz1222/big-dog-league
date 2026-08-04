# -*- coding: utf-8 -*-
"""关闭 estop 诊断的纯软件测试；所有 ros2 调用均由临时 fake 接管。"""

import os
from pathlib import Path
import subprocess

import pytest


WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = WORKSPACE_ROOT / 'src' / 'rk_bringup' / 'scripts'
COMMON_SCRIPT = SCRIPTS_DIR / 'stop_safety_common.sh'
MISSION_STOP_SCRIPT = SCRIPTS_DIR / 'mission_stop.sh'
STOP_LINE_SCRIPT = SCRIPTS_DIR / 'stop_line_system.sh'


def _write_fake_ros2(tmp_path):
    """写入不连接 ROS 的 CLI 替身，并记录所有参数供安全断言。"""
    fake_bin = tmp_path / 'fake_bin'
    fake_bin.mkdir()
    fake_ros2 = fake_bin / 'ros2'
    fake_ros2.write_text(
        '''#!/bin/bash
set -u
printf '%s\\n' "$*" >> "${FAKE_ROS2_CALL_LOG}"
if [ "$1" = service ] && [ "$2" = type ]; then
    case "${FAKE_SERVICE_TYPE_MODE:-ok}" in
        ok) printf '%s\\n' std_srvs/srv/SetBool ;;
        unavailable) printf '%s\\n' 'service graph unavailable' >&2; exit 1 ;;
        wrong_type) printf '%s\\n' std_srvs/srv/Trigger ;;
    esac
    exit 0
fi
if [ "$1" = service ] && [ "$2" = call ]; then
    count=0
    if [ -f "${FAKE_ESTOP_COUNTER}" ]; then
        count="$(cat "${FAKE_ESTOP_COUNTER}")"
    fi
    count=$((count + 1))
    printf '%s' "$count" > "${FAKE_ESTOP_COUNTER}"
    case "${FAKE_ESTOP_MODE:-success}" in
        success)
            printf '%s\\n' 'SetBool_Response(success=True, message="ok")'
            ;;
        yaml) printf '%s\\n' 'response:'; printf '%s\\n' '  success: true' ;;
        rejected) printf '%s\\n' 'success: false' ;;
        malformed) printf '%s\\n' 'response received without result' ;;
        cli_error) printf '%s\\n' 'transport error' >&2; exit 1 ;;
        timeout) printf '%s\\n' 'deadline exceeded' >&2; exit 124 ;;
        fail_then_success)
            if [ "$count" -eq 1 ]; then
                printf '%s\\n' 'first transport error' >&2
                exit 1
            fi
            printf '%s\\n' 'success = True'
            ;;
        fail_twice)
            printf '%s\\n' "attempt ${count} failed" >&2; exit 1
            ;;
    esac
    exit 0
fi
if [ "$1" = node ] && [ "$2" = list ]; then
    printf '%s\\n' /command_mux_node
    exit 0
fi
if [ "$1" = topic ] && [ "$2" = echo ]; then
    if printf '%s\\n' "$*" | grep -Fq /navigation/cmd_vel; then
        printf '%s\\n' 'linear:' '  x: 0.0' '  y: 0.0' '  z: 0.0'
        printf '%s\\n' 'angular:' '  x: 0.0' '  y: 0.0' '  z: 0.0'
    elif [ "${FAKE_ACTION_STATUS_MODE:-terminal}" = terminal ]; then
        printf '%s\\n' '{"state":"IDLE"}'
    fi
    exit 0
fi
if [ "$1" = topic ] && [ "$2" = pub ]; then exit 0; fi
printf '%s\\n' "unexpected fake ros2 invocation: $*" >&2
exit 2
''',
        encoding='utf-8',
    )
    fake_ros2.chmod(0o700)
    return fake_bin


def _fake_environment(tmp_path, fake_bin, **overrides):
    """构造隔离环境，禁止测试命中系统 ros2 或真实服务。"""
    overlay = tmp_path / 'fake_ros_overlay.bash'
    overlay.write_text(
        'export PATH="{}:$PATH"\n'.format(fake_bin),
        encoding='utf-8',
    )
    environment = os.environ.copy()
    environment.update(
        {
            'PATH': '{}:{}'.format(fake_bin, environment['PATH']),
            'RK_ROS_OVERLAY_SETUP': str(overlay),
            'RK_INSPECTION_WS': str(WORKSPACE_ROOT),
            'RK_ROS_DOMAIN_ID': '231',
            'FAKE_ROS2_CALL_LOG': str(tmp_path / 'ros2_calls.log'),
            'FAKE_ESTOP_COUNTER': str(tmp_path / 'estop_counter.txt'),
            'RK_COMPETITION_RUNTIME_DIR': str(tmp_path / 'runtime'),
            'RK_LINE_RUNTIME_DIR': str(tmp_path / 'line_runtime'),
            'RK_COMPETITION_TMUX_SESSION': 'rk_fake_stop_test',
            'RK_LINE_TMUX_SESSION': 'rk_fake_line_test',
        }
    )
    environment.update(overrides)
    return environment


def _run_common(tmp_path, **overrides):
    fake_bin = _write_fake_ros2(tmp_path)
    environment = _fake_environment(tmp_path, fake_bin, **overrides)
    return subprocess.run(
        [
            'bash', '-c',
            'source "$1"; rk_call_mux_estop test_stage',
            'bash', str(COMMON_SCRIPT),
        ],
        check=False,
        text=True,
        capture_output=True,
        env=environment,
    )


@pytest.mark.parametrize(
    ('mode', 'expected_exit', 'classification', 'response_fragment'),
    [
        ('success', 0, 'SUCCESS', 'success=True'),
        ('yaml', 0, 'SUCCESS', 'success: true'),
        ('rejected', 1, 'SERVICE_REJECTED', 'success: false'),
        ('malformed', 1, 'MALFORMED_RESPONSE', 'without result'),
        ('cli_error', 1, 'CLI_ERROR', 'transport error'),
        ('timeout', 1, 'TIMEOUT', 'deadline exceeded'),
    ],
)
def test_estop_diagnostic_classifies_call_outcomes(
    tmp_path, mode, expected_exit, classification, response_fragment
):
    result = _run_common(tmp_path, FAKE_ESTOP_MODE=mode)

    assert result.returncode == expected_exit
    assert 'stage=test_stage service=/safety/estop' in result.stdout
    assert 'operation=service_type' in result.stdout
    assert 'operation=service_call' in result.stdout
    assert 'classification={}'.format(classification) in result.stdout
    assert (
        response_fragment in result.stdout
        or response_fragment in result.stderr
    )


@pytest.mark.parametrize(
    ('type_mode', 'expected_classification'),
    [('unavailable', 'CLI_ERROR'), ('wrong_type', 'SERVICE_TYPE_UNAVAILABLE')],
)
def test_estop_diagnostic_distinguishes_service_type_failures(
    tmp_path, type_mode, expected_classification
):
    result = _run_common(tmp_path, FAKE_SERVICE_TYPE_MODE=type_mode)

    assert result.returncode == 1
    assert 'operation=service_type' in result.stdout
    assert 'classification={}'.format(expected_classification) in result.stdout
    assert 'operation=service_call' not in result.stdout


def _run_stop_line(tmp_path, **overrides):
    fake_bin = _write_fake_ros2(tmp_path)
    environment = _fake_environment(tmp_path, fake_bin, **overrides)
    return subprocess.run(
        ['bash', str(STOP_LINE_SCRIPT)],
        check=False,
        text=True,
        capture_output=True,
        env=environment,
    ), environment


def test_first_estop_failure_second_success_completes_without_fallback(
    tmp_path,
):
    result, environment = _run_stop_line(
        tmp_path, FAKE_ESTOP_MODE='fail_then_success'
    )

    assert result.returncode == 0, result.stderr
    assert 'stage=mission_stop_primary' in result.stdout
    assert 'stage=stop_line_system_retry' in result.stdout
    assert (
        'stage=mission_stop_primary service=/safety/estop'
        in result.stdout
    )
    assert 'classification=CLI_ERROR' in result.stdout
    assert (
        'stage=stop_line_system_retry service=/safety/estop'
        in result.stdout
    )
    assert 'classification=SUCCESS' in result.stdout
    assert (
        'Verified three consecutive command_mux zero outputs.'
        in result.stdout
    )
    assert 'EMERGENCY FALLBACK' not in result.stderr
    calls = Path(environment['FAKE_ROS2_CALL_LOG']).read_text(encoding='utf-8')
    assert '/navigation/cmd_vel geometry_msgs/msg/Twist' not in calls


def test_two_estop_failures_trigger_logged_fallback_only_in_fake_cli(tmp_path):
    result, environment = _run_stop_line(
        tmp_path, FAKE_ESTOP_MODE='fail_twice'
    )

    assert result.returncode == 1
    assert result.stdout.count('classification=CLI_ERROR') == 2
    assert 'EMERGENCY FALLBACK' in result.stderr
    calls = Path(environment['FAKE_ROS2_CALL_LOG']).read_text(encoding='utf-8')
    assert '/navigation/cmd_vel geometry_msgs/msg/Twist' in calls
    assert 'go2_sdk' not in calls


def test_primary_success_with_incomplete_actions_keeps_estop_and_retry(
    tmp_path,
):
    result, environment = _run_stop_line(
        tmp_path,
        FAKE_ESTOP_MODE='success',
        FAKE_ACTION_STATUS_MODE='missing',
        RK_COMPETITION_ACTION_CANCEL_TIMEOUT_SEC='0',
    )

    assert result.returncode == 0, result.stderr
    assert 'white-bar Action did not reach a terminal state' in result.stderr
    assert 'inspection Action did not reach a terminal state' in result.stderr
    assert result.stdout.count('classification=SUCCESS') == 2
    assert (
        'Verified three consecutive command_mux zero outputs.'
        in result.stdout
    )
    assert 'EMERGENCY FALLBACK' not in result.stderr
    calls = Path(environment['FAKE_ROS2_CALL_LOG']).read_text(encoding='utf-8')
    assert '/safety/estop std_srvs/srv/SetBool {data: true}' in calls
    assert 'go2_sdk' not in calls


def test_scripts_share_single_diagnostic_implementation():
    """两处入口只能调用同一函数，避免再次出现不一致的错误吞没。"""
    mission_source = MISSION_STOP_SCRIPT.read_text(encoding='utf-8')
    stop_source = STOP_LINE_SCRIPT.read_text(encoding='utf-8')

    assert 'source "${SCRIPT_DIR}/stop_safety_common.sh"' in mission_source
    assert 'source "${SCRIPT_DIR}/stop_safety_common.sh"' in stop_source
    assert 'rk_call_mux_estop mission_stop_primary' in mission_source
    assert 'rk_call_mux_estop stop_line_system_retry' in stop_source
    assert 'call_mux_estop()' not in mission_source
    assert 'call_mux_estop()' not in stop_source
