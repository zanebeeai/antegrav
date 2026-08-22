#!/usr/bin/env bash
# Switch the Ethon Jetson between CAPTURE and AUTONOMY mode by toggling the two
# mutually-exclusive systemd services (they Conflict= on the CSI cameras).
#
# Invoked by nextion_hmi.py via the HMI mode button:
#     sudo -n /home/jetson/ethon/ethon_set_mode.sh {capture|autonomy}
#
# Needs the /etc/sudoers.d/ethon-hmi drop-in so the jetson user may run THIS
# script (only) without a password. Switching to autonomy merely LAUNCHES the
# stack; the planner boots DISARMED, so the car does not move until armed.
set -euo pipefail

case "${1:-}" in
  autonomy)
    systemctl disable --now ethon-capture.service
    systemctl enable  --now ethon-stack.service
    ;;
  capture)
    systemctl disable --now ethon-stack.service
    systemctl enable  --now ethon-capture.service
    ;;
  *)
    echo "usage: $0 {capture|autonomy}" >&2
    exit 2
    ;;
esac
