#!/usr/bin/env python3

"""(Re)start the RealSense D435i camera with safe cleanup and bounded retry.

This script does NOT perform USB reset, does NOT modify udev, and does NOT
kill unrelated processes.  It is designed to be sourced from ros_clean_env.sh
or called directly in a tmux session.

Usage:
    python3 restart_realsense.py [--max-wait-sec 30]
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time


def _find_realsense_processes():
    """Return list of (pid, cmdline) for running realsense2_camera_node procs."""
    results = []
    try:
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                cmdline_path = os.path.join('/proc', entry, 'cmdline')
                with open(cmdline_path, 'rb') as fh:
                    raw = fh.read()
                text = raw.decode('utf-8', errors='ignore')
                if 'realsense2_camera_node' in text:
                    results.append((int(entry), text.replace('\x00', ' ').strip()))
            except (OSError, PermissionError, ValueError):
                continue
    except OSError:
        pass
    return results


def _usb_device_present(vendor_id='8086', product_id='0b3a'):
    """Check if a RealSense D435i USB device appears in lsusb.

    D435i typically uses Intel vendor 8086:0b3a.
    """
    try:
        output = subprocess.check_output(
            ['lsusb'], stderr=subprocess.DEVNULL, timeout=5,
        ).decode('utf-8', errors='ignore')
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, OSError):
        return False
    needle = '{}:{}'.format(vendor_id.lower(), product_id.lower())
    return needle in output.lower()


def _stop_camera_gracefully(timeout_sec=8.0):
    """Send SIGTERM to realsense2_camera_node processes, then wait."""
    procs = _find_realsense_processes()
    if not procs:
        return True, 'no_realsense_processes_found'

    for pid, _cmdline in procs:
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass

    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        remaining = _find_realsense_processes()
        if not remaining:
            return True, 'stopped_{}_procs'.format(len(procs))
        time.sleep(0.3)

    # Force kill remaining.
    remaining = _find_realsense_processes()
    for pid, _cmdline in remaining:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(0.5)
    final = _find_realsense_processes()
    if final:
        return False, '{}_procs_unresponsive_to_sigkill'.format(len(final))
    return True, 'force_killed_{}_procs'.format(len(remaining))


def _wait_device_ready(timeout_sec=15.0):
    """Poll until the RealSense USB device appears in lsusb."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if _usb_device_present():
            return True
        time.sleep(0.5)
    return False


def main():
    parser = argparse.ArgumentParser(
        description='Safe RealSense camera restart',
    )
    parser.add_argument(
        '--max-wait-sec', type=float, default=30.0,
        help='Maximum total wait for device to become ready.',
    )
    parser.add_argument(
        '--stop-only', action='store_true',
        help='Only stop the camera, do not restart.',
    )
    args = parser.parse_args()

    print('[camera] Stopping existing RealSense processes ...')
    ok, detail = _stop_camera_gracefully()
    print('[camera] Stop result: {} ({})'.format('OK' if ok else 'FAIL', detail))

    if args.stop_only:
        if ok:
            print('[camera] Camera stopped. Physical USB replug may be needed '
                  'before next start.')
        sys.exit(0 if ok else 1)

    # Wait for USB device to settle.
    print('[camera] Waiting for USB device (max {:.0f}s) ...'.format(
        args.max_wait_sec))
    if not _wait_device_ready(args.max_wait_sec):
        print(
            '[camera] ERROR: RealSense USB device not detected after '
            '{:.0f}s.'.format(args.max_wait_sec),
        )
        print(
            '[camera] The camera may need physical USB replug. '
            'Do NOT start the camera node until the device appears in lsusb '
            '(8086:0b3a).',
        )
        sys.exit(1)

    print('[camera] USB device present. Camera can be restarted by the '
          'launch file.')
    sys.exit(0)


if __name__ == '__main__':
    main()
