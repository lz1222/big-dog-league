#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

if command -v rosdep >/dev/null 2>&1; then
  rosdep install --from-paths src --ignore-src -r -y
else
  echo "rosdep not found, skipping dependency installation"
fi

colcon build --symlink-install
