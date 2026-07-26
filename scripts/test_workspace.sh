#!/usr/bin/env bash

set -Eeuo pipefail

RK_TEST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RK_TEST_TMP_ROOT=""
RK_TEST_PASS=0
RK_TEST_WARN=0
RK_TEST_FAIL=0
RK_TEST_SKIP=0
RK_TEST_LAST_STATUS=0
RK_TEST_ROS_READY=0
RK_TEST_BUILD_OK=0

report() {
  local level="$1"
  local label="$2"
  local detail="${3:-}"
  local suffix=""
  if [[ -n "$detail" ]]; then
    suffix=" - $detail"
  fi
  printf '[%s] %s%s\n' "$level" "$label" "$suffix"
  case "$level" in
    PASS) ((RK_TEST_PASS += 1)) ;;
    WARN) ((RK_TEST_WARN += 1)) ;;
    FAIL) ((RK_TEST_FAIL += 1)) ;;
    SKIP) ((RK_TEST_SKIP += 1)) ;;
  esac
}

show_log_tail() {
  local log_path="$1"
  if [[ -s "$log_path" ]]; then
    printf '  log: %s\n' "$log_path"
    tail -n 16 "$log_path" | sed 's/^/  | /'
  fi
}

run_step() {
  local label="$1"
  local log_path="$2"
  shift 2
  if "$@" >"$log_path" 2>&1; then
    RK_TEST_LAST_STATUS=0
    report PASS "$label"
  else
    RK_TEST_LAST_STATUS=$?
    report FAIL "$label" "exit=$RK_TEST_LAST_STATUS"
    show_log_tail "$log_path"
  fi
  return 0
}

cleanup_temp() {
  if [[ -z "$RK_TEST_TMP_ROOT" || ! -d "$RK_TEST_TMP_ROOT" ]]; then
    return
  fi
  if [[ "${RK_KEEP_TEST_ARTIFACTS:-0}" == "1" ]]; then
    printf '[WARN] Temporary test artifacts retained - %s\n' "$RK_TEST_TMP_ROOT"
    return
  fi
  case "$RK_TEST_TMP_ROOT" in
    /tmp/rk_workspace_test.*)
      rm -rf -- "$RK_TEST_TMP_ROOT"
      ;;
    *)
      printf '[WARN] Refused to remove unexpected temporary path - %s\n' \
        "$RK_TEST_TMP_ROOT"
      ;;
  esac
}

finish() {
  printf '\nWorkspace test summary: PASS=%d WARN=%d FAIL=%d SKIP=%d\n' \
    "$RK_TEST_PASS" "$RK_TEST_WARN" "$RK_TEST_FAIL" "$RK_TEST_SKIP"
  if ((RK_TEST_FAIL > 0)); then
    return 1
  fi
  return 0
}

detect_ros() {
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    RK_TEST_ROS_READY=1
    report PASS "ROS environment" "ROS_DISTRO=$ROS_DISTRO (already sourced)"
    return
  fi

  local architecture
  architecture="$(uname -m)"
  local -a candidates=()
  if [[ "$architecture" == "aarch64" ]]; then
    candidates=(/opt/ros/foxy/setup.bash /opt/ros/humble/setup.bash)
  else
    candidates=(/opt/ros/humble/setup.bash /opt/ros/foxy/setup.bash)
  fi

  local setup_path
  for setup_path in "${candidates[@]}"; do
    [[ -f "$setup_path" ]] || continue
    set +u
    if source "$setup_path"; then
      set -u
      if command -v ros2 >/dev/null 2>&1; then
        RK_TEST_ROS_READY=1
        report PASS "ROS environment" \
          "auto-sourced ${ROS_DISTRO:-unknown} from $setup_path"
        return
      fi
    else
      set -u
    fi
  done

  report SKIP "ROS environment" \
    "no usable /opt/ros setup found; ROS build and package tests will be skipped"
}

python_syntax_check() {
  env PYTHONPYCACHEPREFIX="$RK_TEST_TMP_ROOT/pycache" \
    python3 -m compileall -q "$RK_TEST_ROOT/src" "$RK_TEST_ROOT/scripts"
}

shell_syntax_check() {
  local script_path
  while IFS= read -r -d '' script_path; do
    bash -n "$RK_TEST_ROOT/$script_path"
  done < <(git -C "$RK_TEST_ROOT" ls-files -z -- '*.sh')
}

package_pythonpath() {
  local combined=""
  local package_root
  for package_root in "$RK_TEST_ROOT"/src/*; do
    [[ -d "$package_root" ]] || continue
    if [[ -f "$package_root/$(basename "$package_root")/__init__.py" ]]; then
      if [[ -n "$combined" ]]; then
        combined+=":"
      fi
      combined+="$package_root"
    fi
  done
  printf '%s' "$combined"
}

run_per_package_pytests() {
  local workspace_pythonpath="$1"
  local package_root
  for package_root in "$RK_TEST_ROOT"/src/*; do
    [[ -d "$package_root/test" ]] || continue
    local package_name
    package_name="$(basename "$package_root")"
    local package_log="$RK_TEST_TMP_ROOT/pytest-${package_name}.log"
    if (
      cd "$package_root"
      env \
        PYTHONDONTWRITEBYTECODE=1 \
        PYTHONPATH="${workspace_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" \
        timeout 120s python3 -m pytest -q \
          -p no:cacheprovider \
          --import-mode=importlib \
          test
    ) >"$package_log" 2>&1; then
      report PASS "pytest package: $package_name"
    else
      local status=$?
      report FAIL "pytest package: $package_name" "exit=$status"
      show_log_tail "$package_log"
    fi
  done
}

safe_pytest_collection() {
  local workspace_pythonpath="$1"
  env \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONPATH="${workspace_pythonpath}${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest --collect-only -q \
      -p no:cacheprovider \
      -p no:launch_testing \
      -p no:launch_ros \
      --import-mode=importlib \
      "$RK_TEST_ROOT/src"
}

colcon_build_clean() {
  colcon --log-base "$RK_TEST_TMP_ROOT/log" build \
    --base-paths "$RK_TEST_ROOT/src" \
    --symlink-install \
    --build-base "$RK_TEST_TMP_ROOT/build" \
    --install-base "$RK_TEST_TMP_ROOT/install"
}

colcon_test_clean() {
  colcon --log-base "$RK_TEST_TMP_ROOT/log" test \
    --base-paths "$RK_TEST_ROOT/src" \
    --build-base "$RK_TEST_TMP_ROOT/build" \
    --install-base "$RK_TEST_TMP_ROOT/install"
}

colcon_result_clean() {
  colcon test-result \
    --test-result-base "$RK_TEST_TMP_ROOT/build" \
    --verbose
}

printf 'Repository Hygiene workspace test: %s\n' "$RK_TEST_ROOT"

if ! command -v git >/dev/null 2>&1; then
  report FAIL "Git availability" "git is not installed"
  finish
  exit $?
fi

RK_TEST_TMP_ROOT="$(mktemp -d /tmp/rk_workspace_test.XXXXXX)"
trap cleanup_temp EXIT
mkdir -p "$RK_TEST_TMP_ROOT/log"

git -C "$RK_TEST_ROOT" status --porcelain=v1 -z \
  >"$RK_TEST_TMP_ROOT/git-status-before"
status_count="$(tr -cd '\0' <"$RK_TEST_TMP_ROOT/git-status-before" | wc -c)"
if [[ "$status_count" == "0" ]]; then
  report PASS "Git working tree" "clean"
else
  report WARN "Git working tree" \
    "$status_count existing changes; test will verify it adds none"
fi

if [[ -n "$(git -C "$RK_TEST_ROOT" ls-files build install log)" ]]; then
  report FAIL "Root generated paths in Git index"
else
  report PASS "Root generated paths in Git index" "none"
fi

if command -v python3 >/dev/null 2>&1; then
  run_step "Python syntax" "$RK_TEST_TMP_ROOT/python-syntax.log" \
    python_syntax_check
else
  report FAIL "Python syntax" "python3 is not installed"
fi

run_step "Shell syntax" "$RK_TEST_TMP_ROOT/shell-syntax.log" \
  shell_syntax_check

detect_ros

workspace_pythonpath="$(package_pythonpath)"
if python3 -c 'import pytest' >/dev/null 2>&1; then
  run_step "Pure Python perception tests" \
    "$RK_TEST_TMP_ROOT/pytest-perception.log" \
    env \
      PYTHONDONTWRITEBYTECODE=1 \
      PYTHONPATH="$RK_TEST_ROOT/src/rk_perception${PYTHONPATH:+:$PYTHONPATH}" \
      python3 -m pytest -q \
        -p no:cacheprovider \
        "$RK_TEST_ROOT/src/rk_perception/test/test_real_line_tracker_node.py" \
        "$RK_TEST_ROOT/src/rk_perception/test/test_real_sign_detector_node.py"

  if ((RK_TEST_ROS_READY == 1)); then
    run_step "ROS Python cmd_vel bridge tests" \
      "$RK_TEST_TMP_ROOT/pytest-bridge.log" \
      env \
        PYTHONDONTWRITEBYTECODE=1 \
        PYTHONPATH="$RK_TEST_ROOT/src/rk_unitree_driver${PYTHONPATH:+:$PYTHONPATH}" \
        python3 -m pytest -q \
          -p no:cacheprovider \
          "$RK_TEST_ROOT/src/rk_unitree_driver/test/test_cmd_vel_bridge_node.py"

    run_step "Safe pytest collection under src" \
      "$RK_TEST_TMP_ROOT/pytest-collect.log" \
      safe_pytest_collection "$workspace_pythonpath"
    if ((RK_TEST_LAST_STATUS == 0)); then
      tail -n 1 "$RK_TEST_TMP_ROOT/pytest-collect.log" | sed 's/^/  /'
    fi

    run_per_package_pytests "$workspace_pythonpath"
  else
    report SKIP "ROS Python cmd_vel bridge tests" "ROS environment unavailable"
    report SKIP "Safe pytest collection under src" "ROS environment unavailable"
    report SKIP "Per-package pytest" "ROS environment unavailable"
  fi
else
  report SKIP "Python tests" "python3 pytest module is unavailable"
  report SKIP "Safe pytest collection under src" "pytest is unavailable"
  report SKIP "Per-package pytest" "pytest is unavailable"
fi

report SKIP "Raw repository-wide pytest collection" \
  "third_party D1 test has import-time hardware side effects; see docs/TEST_BASELINE.md"

if ((RK_TEST_ROS_READY == 1)) && command -v colcon >/dev/null 2>&1; then
  run_step "Clean colcon build --symlink-install" \
    "$RK_TEST_TMP_ROOT/colcon-build.log" \
    colcon_build_clean
  if ((RK_TEST_LAST_STATUS == 0)); then
    RK_TEST_BUILD_OK=1
    run_step "colcon test" "$RK_TEST_TMP_ROOT/colcon-test.log" \
      colcon_test_clean
    run_step "colcon test-result --verbose" \
      "$RK_TEST_TMP_ROOT/colcon-test-result.log" \
      colcon_result_clean
    if ((RK_TEST_LAST_STATUS == 0)); then
      tail -n 2 "$RK_TEST_TMP_ROOT/colcon-test-result.log" | sed 's/^/  /'
    fi
  else
    report SKIP "colcon test" "clean build failed"
    report SKIP "colcon test-result --verbose" "clean build failed"
  fi
else
  report SKIP "Clean colcon build --symlink-install" \
    "ROS environment or colcon unavailable"
  report SKIP "colcon test" "build prerequisite unavailable"
  report SKIP "colcon test-result --verbose" "build prerequisite unavailable"
fi

run_step "National preflight" "$RK_TEST_TMP_ROOT/national-preflight.log" \
  bash "$RK_TEST_ROOT/scripts/national_preflight.sh"

report SKIP "Unitree Go2/Foxy hardware validation" \
  "requires robot-side Foxy, SDK/UDP runtime, network, and safety operator"
report SKIP "D1 arm hardware validation" \
  "never automated; requires vendor libraries, hardware ACK, and safety procedure"
report SKIP "Camera and physical-task validation" \
  "requires real camera frames, course, obstacles, payloads, and video evidence"

git -C "$RK_TEST_ROOT" status --porcelain=v1 -z \
  >"$RK_TEST_TMP_ROOT/git-status-after"
if cmp -s \
  "$RK_TEST_TMP_ROOT/git-status-before" \
  "$RK_TEST_TMP_ROOT/git-status-after"; then
  report PASS "Test run repository cleanliness" "no Git-visible files added"
else
  report FAIL "Test run repository cleanliness" \
    "git status changed during the test run"
  diff \
    <(tr '\0' '\n' <"$RK_TEST_TMP_ROOT/git-status-before") \
    <(tr '\0' '\n' <"$RK_TEST_TMP_ROOT/git-status-after") \
    >"$RK_TEST_TMP_ROOT/git-status.diff" || true
  show_log_tail "$RK_TEST_TMP_ROOT/git-status.diff"
fi

finish
