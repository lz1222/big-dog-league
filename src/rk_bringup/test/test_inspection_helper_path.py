"""纯软件测试：正式 inspection helper 必须来自安装树绝对可执行路径。"""

import os

import pytest

from rk_bringup.inspection_helper_path import InspectionHelperPathError
from rk_bringup.inspection_helper_path import SDK_HELPER_BASENAME
from rk_bringup.inspection_helper_path import SDK_HELPER_PACKAGE
from rk_bringup.inspection_helper_path import resolve_production_sdk_action_helper
from rk_bringup.inspection_helper_path import select_sdk_action_helper
from rk_bringup.inspection_helper_path import validate_absolute_executable


def _write_executable(path):
    path.write_text('#!/bin/sh\nexit 0\n', encoding='utf-8')
    path.chmod(0o755)
    return str(path)


def _production_helper(tmp_path):
    helper_dir = tmp_path / 'lib' / SDK_HELPER_PACKAGE
    helper_dir.mkdir(parents=True)
    return _write_executable(helper_dir / SDK_HELPER_BASENAME)


def test_production_helper_resolves_from_absolute_install_prefix(tmp_path):
    expected = _production_helper(tmp_path)

    resolved = resolve_production_sdk_action_helper(str(tmp_path))

    assert resolved == expected
    assert os.path.isabs(resolved)
    assert os.path.isfile(resolved)
    assert os.access(resolved, os.X_OK)


def test_production_resolution_preserves_symlink_install_tree_path(tmp_path):
    target = _write_executable(tmp_path / 'build_helper')
    helper_dir = tmp_path / 'install' / 'lib' / SDK_HELPER_PACKAGE
    helper_dir.mkdir(parents=True)
    install_helper = helper_dir / SDK_HELPER_BASENAME
    install_helper.symlink_to(target)

    resolved = resolve_production_sdk_action_helper(str(tmp_path / 'install'))

    assert resolved == str(install_helper)
    assert '/install/lib/' in resolved


@pytest.mark.parametrize('configured', ('go2_sdk_motion_action', './helper'))
def test_basename_and_relative_helper_paths_are_rejected(configured):
    with pytest.raises(InspectionHelperPathError, match='must_be_absolute'):
        validate_absolute_executable(configured)


def test_missing_or_non_executable_helper_fails_closed(tmp_path):
    missing = tmp_path / 'missing_helper'
    with pytest.raises(InspectionHelperPathError, match='not_file'):
        validate_absolute_executable(str(missing))

    non_executable = tmp_path / 'not_executable'
    non_executable.write_text('#!/bin/sh\n', encoding='utf-8')
    non_executable.chmod(0o644)
    with pytest.raises(InspectionHelperPathError, match='not_executable'):
        validate_absolute_executable(str(non_executable))


def test_software_smoke_uses_fake_without_resolving_real_install_tree(tmp_path):
    fake = _write_executable(tmp_path / 'fake_sdk_motion_helper')

    selected = select_sdk_action_helper(
        'false', 'true', fake, 'relative-production-prefix-is-not-used'
    )

    assert selected == os.path.realpath(fake)


def test_production_selection_rejects_invalid_package_prefix(tmp_path):
    fake = _write_executable(tmp_path / 'fake_sdk_motion_helper')
    with pytest.raises(
        InspectionHelperPathError, match='package_prefix_must_be_absolute'
    ):
        select_sdk_action_helper('true', 'false', fake, 'relative-prefix')
