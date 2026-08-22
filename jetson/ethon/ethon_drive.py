#!/usr/bin/env python3
"""Ethon drive controller entry point.

Implementation lives in drive/ (config, can_bus, torque_map, pedal,
steering, node) — see drive/node.py for the full architecture docstring,
safety model, and topic list, and drive/pedal.py for the two manual-drive
pedal modes ("one_pedal" default / "coast", switchable from the dashboard).

Kept as a top-level script (not an installed ROS2 package) because
ethon-drive.service execs this file directly: `python3
/home/jetson/ethon/ethon_drive.py`. See PROJECT_HANDOFF.md.
"""
from drive.node import main

if __name__ == "__main__":
    main()
