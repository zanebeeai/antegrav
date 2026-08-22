#!/usr/bin/env bash
# Clear a latched E-STOP on the Ethon EV.
#
# Two separate processes each latch their own estop state and only clear it
# on restart (a deliberate safety design -- a stale `/ethon/estop false` must
# not silently re-arm the motors):
#   - ethon_drive.py (its own systemd unit, ethon-drive.service, since
#     2026-08-16 -- it used to run inside ethon-stack.launch.py, but was split
#     out so steering re-homes don't also kill/reinit the cameras)
#   - health_monitor.py's own estop reflex latch (self._estop_fired), still
#     inside ethon-stack.service
# Both must be bounced or the latch only half-clears.
#
# Callers (wheel_bridge.py ARM+DISARM hold, or the web dashboard CLEAR E-STOP
# button) publish `/ethon/estop false` FIRST, so both restarted processes come
# up with a cleared latched topic value.
#
# Invoked as:  sudo -n /home/jetson/ethon/ethon_clear_estop.sh
# Needs the /etc/sudoers.d/ethon-clear-estop drop-in (NOPASSWD, this script only).
set -euo pipefail

systemctl restart ethon-drive.service
systemctl restart ethon-stack.service
