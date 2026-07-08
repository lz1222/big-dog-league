#!/usr/bin/env bash
set -u

usage() {
  cat <<'EOF'
Usage:
  go2_warning_sign_sdk_loop.sh [network_interface] [options]

Options:
  --dry-run                 Only print the mapped action; do not run it.
  --timeout-sec SEC         Stop trying after SEC seconds. Default: 20.
  --interval-sec SEC        Sleep between attempts. Default: 0.35.
  --min-confidence VALUE    Minimum detection confidence. Default: 0.50.
  --template-score VALUE    Minimum template score. Default: 0.10.
  --action-wait-sec SEC     Wait time passed to SDK action. Default: 3.0.
  --image PATH              Reuse this image path. Default: /tmp/go2_warning_sign.jpg.
  -h, --help                Show this help.

Examples:
  go2_warning_sign_sdk_loop.sh eth0 --dry-run
  go2_warning_sign_sdk_loop.sh eth0
EOF
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_WS="/home/unitree/rk_inspection_ws"
if [[ -d "${RK_WS:-}" ]]; then
  WS="$RK_WS"
elif [[ -d "$DEFAULT_WS" ]]; then
  WS="$DEFAULT_WS"
else
  WS="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi

INTERFACE="eth0"
if [[ $# -gt 0 && "${1:-}" != -* ]]; then
  INTERFACE="$1"
  shift
fi

DRY_RUN=false
TIMEOUT_SEC=20
INTERVAL_SEC=0.35
MIN_CONFIDENCE=0.50
TEMPLATE_SCORE=0.10
ACTION_WAIT_SEC=3.0
IMAGE_PATH="/tmp/go2_warning_sign.jpg"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --timeout-sec)
      TIMEOUT_SEC="$2"
      shift 2
      ;;
    --interval-sec)
      INTERVAL_SEC="$2"
      shift 2
      ;;
    --min-confidence)
      MIN_CONFIDENCE="$2"
      shift 2
      ;;
    --template-score)
      TEMPLATE_SCORE="$2"
      shift 2
      ;;
    --action-wait-sec)
      ACTION_WAIT_SEC="$2"
      shift 2
      ;;
    --image)
      IMAGE_PATH="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

ARCH="$(uname -m)"
case "$ARCH" in
  aarch64|arm64)
    SDK_ARCH="aarch64"
    ;;
  x86_64|amd64)
    SDK_ARCH="x86_64"
    ;;
  *)
    SDK_ARCH="$ARCH"
    ;;
esac

export LD_LIBRARY_PATH="$WS/third_party/unitree_sdk2_official/thirdparty/lib/$SDK_ARCH:$WS/install/rk_go2_sdk_bridge/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$WS/src/rk_perception:${PYTHONPATH:-}"

CAPTURE_TOOL="$WS/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_capture_image"
ACTION_TOOL="$WS/install/rk_go2_sdk_bridge/lib/rk_go2_sdk_bridge/go2_sdk_motion_action"
CLASSIFIER="$WS/src/rk_go2_sdk_bridge/scripts/warning_sign_image_classifier.py"

if [[ ! -x "$CAPTURE_TOOL" ]]; then
  echo "Missing executable: $CAPTURE_TOOL" >&2
  echo "Build rk_go2_sdk_bridge first." >&2
  exit 1
fi
if [[ ! -x "$ACTION_TOOL" ]]; then
  echo "Missing executable: $ACTION_TOOL" >&2
  echo "Build rk_go2_sdk_bridge first." >&2
  exit 1
fi
if [[ ! -f "$CLASSIFIER" ]]; then
  echo "Missing classifier: $CLASSIFIER" >&2
  exit 1
fi

stop_robot() {
  "$ACTION_TOOL" "$INTERFACE" stop_move 0.2 >/dev/null 2>&1 || true
}
trap 'echo "Interrupted, sending stop_move..."; stop_robot; exit 130' INT TERM

echo "Go2 SDK warning-sign action loop"
echo "workspace=$WS"
echo "interface=$INTERFACE"
echo "dry_run=$DRY_RUN"
echo "timeout_sec=$TIMEOUT_SEC"
echo "image=$IMAGE_PATH"

START_TIME="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"

while true; do
  NOW="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
  ELAPSED="$(python3 - "$START_TIME" "$NOW" <<'PY'
import sys
print(float(sys.argv[2]) - float(sys.argv[1]))
PY
)"
  EXPIRED="$(python3 - "$ELAPSED" "$TIMEOUT_SEC" <<'PY'
import sys
print('1' if float(sys.argv[1]) >= float(sys.argv[2]) else '0')
PY
)"
  if [[ "$EXPIRED" == "1" ]]; then
    echo "No warning sign detected before timeout."
    exit 2
  fi

  if ! "$CAPTURE_TOOL" "$INTERFACE" "$IMAGE_PATH" 4 120 >/tmp/go2_capture.log 2>&1; then
    echo "capture failed: $(tail -n 1 /tmp/go2_capture.log)"
    sleep "$INTERVAL_SEC"
    continue
  fi

  DETECTION_JSON="$(python3 "$CLASSIFIER" "$IMAGE_PATH" \
    --min-confidence "$MIN_CONFIDENCE" \
    --template-min-score "$TEMPLATE_SCORE" 2>/tmp/go2_warning_classifier.err)"
  CLASSIFIER_RC=$?

  if [[ "$CLASSIFIER_RC" -eq 2 ]]; then
    echo "no warning sign yet, elapsed=${ELAPSED}s"
    sleep "$INTERVAL_SEC"
    continue
  fi
  if [[ "$CLASSIFIER_RC" -ne 0 ]]; then
    echo "classifier failed: $DETECTION_JSON" >&2
    cat /tmp/go2_warning_classifier.err >&2 || true
    exit "$CLASSIFIER_RC"
  fi

  ACTION_NAME="$(python3 - "$DETECTION_JSON" <<'PY'
import json
import sys
print(json.loads(sys.argv[1]).get('action', ''))
PY
)"
  SIGN_VALUE="$(python3 - "$DETECTION_JSON" <<'PY'
import json
import sys
print(json.loads(sys.argv[1]).get('sign_value', ''))
PY
)"
  CONFIDENCE="$(python3 - "$DETECTION_JSON" <<'PY'
import json
import sys
print(json.loads(sys.argv[1]).get('confidence', ''))
PY
)"

  echo "detected sign=$SIGN_VALUE confidence=$CONFIDENCE action=$ACTION_NAME"
  if [[ -z "$ACTION_NAME" ]]; then
    echo "No mapped SDK action for sign: $SIGN_VALUE" >&2
    exit 3
  fi

  if [[ "$DRY_RUN" == "true" ]]; then
    echo "[DRY_RUN] $ACTION_TOOL $INTERFACE $ACTION_NAME $ACTION_WAIT_SEC"
    exit 0
  fi

  "$ACTION_TOOL" "$INTERFACE" "$ACTION_NAME" "$ACTION_WAIT_SEC"
  exit $?
done
