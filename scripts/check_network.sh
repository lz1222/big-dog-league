#!/usr/bin/env bash
set -euo pipefail

echo "Host network interfaces:"
ip -brief address || true

if [ "$#" -gt 0 ]; then
  ROBOT_IP="$1"
  echo "Checking robot IP: ${ROBOT_IP}"
  ping -c 3 "${ROBOT_IP}"
else
  echo "No robot IP provided. Usage: ./scripts/check_network.sh <robot-ip>"
fi
