"""正式 ROS clean environment 的父 shell 污染回归。"""

import os
from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
ENV_SCRIPT = PACKAGE_ROOT / 'scripts' / 'ros_clean_env.sh'


def test_clean_env_forces_foxy_domain10_and_removes_cyclonedds_pollution():
    environment = os.environ.copy()
    environment.update({
        'RK_INSPECTION_WS': str(WORKSPACE_ROOT),
        'RK_ROS_DOMAIN_ID': '10',
        'ROS_DISTRO': 'humble',
        'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
        'CYCLONEDDS_URI': 'file:///tmp/poison.xml',
        'CYCLONEDDS_HOME': '/tmp/poison-home',
        'LD_LIBRARY_PATH': ':'.join((
            '/usr/local/cyclonedds/lib',
            '/home/unitree/cyclonedds_ws/install/cyclonedds/lib',
            '/usr/local/cuda/lib64',
            '/opt/keep-me/lib',
        )),
    })
    command = r'''
set -euo pipefail
source "$1"
printf 'ROS_DISTRO=%s\n' "$ROS_DISTRO"
printf 'ROS_DOMAIN_ID=%s\n' "$ROS_DOMAIN_ID"
printf 'RMW=%s\n' "${RMW_IMPLEMENTATION-unset}"
printf 'URI=%s\n' "${CYCLONEDDS_URI-unset}"
printf 'HOME_VAR=%s\n' "${CYCLONEDDS_HOME-unset}"
printf 'LD=%s\n' "${LD_LIBRARY_PATH-}"
'''
    result = subprocess.run(
        ['bash', '-c', command, 'bash', str(ENV_SCRIPT)],
        env=environment,
        text=True,
        capture_output=True,
        check=True,
    )
    output = result.stdout
    assert 'ROS_DISTRO=foxy' in output
    assert 'ROS_DOMAIN_ID=10' in output
    assert 'RMW=unset' in output
    assert 'URI=unset' in output
    assert 'HOME_VAR=unset' in output
    assert '/usr/local/cyclonedds/lib' not in output
    assert '/home/unitree/cyclonedds_ws/install/cyclonedds/lib' not in output
    assert '/usr/local/cuda/lib64' in output
    assert '/opt/keep-me/lib' in output


def test_clean_env_has_no_humble_fallback():
    source = ENV_SCRIPT.read_text(encoding='utf-8')
    assert '/opt/ros/foxy/setup.bash' in source
    assert '/opt/ros/humble/setup.bash' not in source
