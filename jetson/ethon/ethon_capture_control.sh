#!/bin/bash
# Narrow root helper used by the wheel's dedicated capture button.
set -euo pipefail

case "${1:-}" in
  start)
    systemctl start ethon-v1-capture.service
    ;;
  stop)
    systemctl stop ethon-v1-capture.service
    ;;
  toggle)
    if systemctl is-active --quiet ethon-v1-capture.service; then
      systemctl stop ethon-v1-capture.service
    else
      systemctl start ethon-v1-capture.service
    fi
    ;;
  *)
    echo "usage: $0 {start|stop|toggle}" >&2
    exit 2
    ;;
esac
