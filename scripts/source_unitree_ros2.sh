#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "This script must be sourced:"
  echo "  source scripts/source_unitree_ros2.sh [iface]"
  exit 1
fi

_rk_source_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_rk_ws_dir="$(cd "${_rk_source_dir}/.." && pwd)"
_rk_iface="${1:-lo}"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source /opt/ros/foxy/setup.bash
else
  echo "Error: no supported ROS 2 setup.bash found under /opt/ros"
  return 1
fi

if [[ -f "${_rk_ws_dir}/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash" ]]; then
  source "${_rk_ws_dir}/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash"
else
  echo "Warning: Unitree ROS2 install setup not found:"
  echo "  ${_rk_ws_dir}/third_party/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

if [[ -f "${_rk_ws_dir}/install/setup.bash" ]]; then
  source "${_rk_ws_dir}/install/setup.bash"
else
  echo "Warning: RK workspace install setup not found:"
  echo "  ${_rk_ws_dir}/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ -d "${_rk_ws_dir}/third_party/unitree_sdk2/install" ]]; then
  export CMAKE_PREFIX_PATH="${_rk_ws_dir}/third_party/unitree_sdk2/install:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${_rk_ws_dir}/third_party/unitree_sdk2/install/lib:${LD_LIBRARY_PATH:-}"
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-${_rk_ws_dir}/log/ros}"
mkdir -p "${ROS_LOG_DIR}"

# Use the older NetworkInterfaceAddress element because the Go2 onboard image
# commonly runs ROS 2 Foxy with a CycloneDDS version that rejects Interfaces.
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${_rk_iface}</NetworkInterfaceAddress></General></Domain></CycloneDDS>"

echo "Unitree ROS2 environment sourced."
echo "  workspace: ${_rk_ws_dir}"
echo "  ROS_DISTRO=${ROS_DISTRO:-unknown}"
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  ROS_LOG_DIR=${ROS_LOG_DIR}"
echo "  CycloneDDS interface: ${_rk_iface}"

unset _rk_source_dir
unset _rk_ws_dir
unset _rk_iface
